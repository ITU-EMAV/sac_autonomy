// NDT kurtarma watchdog'u.
//
// PROBLEM (ndt_failure_diag bag'inde olculdu):
// NDT bir kez yayin yapmayi birakinca kendi kendini kilitliyor. Zincir:
//
//   NDT converge olmuyor -> poz yayinlamiyor -> EKF olu hesaba dusuyor
//   -> NDT'nin baslangic tahmini (EKF'den gelir) kayiyor -> daha da
//   converge olamiyor -> ...
//
// Olculen: t=389.8'den sonra 100 saniye boyunca tek bir NDT pozu yok;
// skipping_publish_num 275'e kadar cikti; EKF pozu ~5.25 m kaydi. Tek
// kurtulus yolu elle RViz'den initial pose vermekti.
//
// Bu dugum onu otomatiklestirir: /diagnostics uzerinden
// "ndt_scan_matcher: scan_matching_status" -> skipping_publish_num izlenir;
// esigi asip belirli bir sure orada kalirsa, mevcut EKF pozu GENIS bir
// kovaryansla /localization/initialize servisine gonderilir. Servis NDT
// align'i o kovaryansin tanimladigi bolgede aratir, yani birkac metrelik
// kaymayi toparlayabilir.
//
// UYARI: initialize servisi align suresince EKF ve NDT'yi deaktive eder
// (bu kurulumda ~5 s). O sure boyunca lokalizasyon YOKTUR. Bu yuzden
// esik ve cooldown bilerek muhafazakar secildi; watchdog son care olarak
// tasarlandi, rutin bir duzeltici degil.

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware/component_interface_specs/localization.hpp>

#include <algorithm>
#include <string>

using InitializeService =
    autoware::component_interface_specs::localization::Initialize::Service;

class NdtRecoveryWatchdogNode : public rclcpp::Node
{
public:
  NdtRecoveryWatchdogNode()
      : Node("ndt_recovery_watchdog")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    status_name_ = declare_parameter<std::string>(
        "status_name", "ndt_scan_matcher: scan_matching_status");
    key_name_ = declare_parameter<std::string>("key_name", "skipping_publish_num");
    ekf_pose_topic_ = declare_parameter<std::string>(
        "ekf_pose_topic", "/localization/ekf_localizer/pose_with_cov");
    initialize_service_ = declare_parameter<std::string>(
        "initialize_service", "/localization/initialize");

    // Kac ardisik reddedilen tahminden sonra kurtarma dusunulur.
    // NDT ~7 Hz calisiyor, yani 20 ~= 3 saniyelik kesintisiz basarisizlik.
    // Bag'de saglikli surus boyunca bu sayac hic 1'i gecmedi; 20 guvenli.
    threshold_ = declare_parameter<int>("skipping_publish_threshold", 20);

    // Esik asildiktan sonra kurtarmayi tetiklemeden once beklenen sure.
    // Gecici bir tikanmanin kendi kendine duzelmesine sans tanir.
    hold_sec_ = declare_parameter<double>("hold_sec", 3.0);

    // Iki kurtarma denemesi arasindaki asgari sure. Align ~5 s surer ve o
    // sure boyunca lokalizasyon yoktur; art arda tetiklenmesi felakettir.
    cooldown_sec_ = declare_parameter<double>("cooldown_sec", 30.0);

    // Align'in arayacagi bolgenin yaricapi. Servis, gonderilen kovaryanstan
    // ornekleyerek aday pozlar uretir. Olculen kayma ~5 m oldugu icin
    // varsayilan 5 m; cok buyutmek align suresini ve yanlis yere oturma
    // riskini artirir.
    search_stddev_m_ = declare_parameter<double>("search_stddev_m", 5.0);
    search_stddev_yaw_ = declare_parameter<double>("search_stddev_yaw_rad", 0.26);  // ~15 derece

    // 0=AUTO (NDT align ile haritaya oturt), 1=DIRECT (hizalamadan resetle).
    // Kurtarmada AUTO sart: zaten kaymis bir pozu dogrudan yazmak ise yaramaz.
    initialize_method_ = declare_parameter<int>("initialize_method", 0);

    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "NDT recovery watchdog is DISABLED (enabled:=false)");
    }
    RCLCPP_INFO(get_logger(), "ndt_recovery_watchdog started");
    RCLCPP_INFO(get_logger(), "  threshold=%d  hold=%.1fs  cooldown=%.1fs  search_radius=%.1fm",
                threshold_, hold_sec_, cooldown_sec_, search_stddev_m_);

    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(50),
        std::bind(&NdtRecoveryWatchdogNode::onDiagnostics, this, std::placeholders::_1));

    // EKF pozu icin QoS'u yayinciyla ayni tut (reliable, keep_last).
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        ekf_pose_topic_, rclcpp::QoS(5),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          last_pose_ = msg;
        });

    client_ = create_client<InitializeService>(initialize_service_);
  }

private:
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for (const auto & status : msg->status) {
      if (status.name != status_name_) continue;

      for (const auto & kv : status.values) {
        if (kv.key != key_name_) continue;

        int skipping = 0;
        try {
          skipping = std::stoi(kv.value);
        } catch (const std::exception &) {
          return;  // "nan" vb. -> yok say
        }
        evaluate(skipping);
        return;
      }
    }
  }

  void evaluate(int skipping)
  {
    const rclcpp::Time now = this->now();

    if (skipping < threshold_) {
      // Saglikli (ya da toparladi): bekleme sayacini sifirla.
      if (breach_started_.nanoseconds() != 0) {
        RCLCPP_INFO(get_logger(), "NDT recovered (skipping_publish_num=%d); monitoring state reset.",
                    skipping);
      }
      breach_started_ = rclcpp::Time(0, 0, now.get_clock_type());
      return;
    }

    if (breach_started_.nanoseconds() == 0) {
      breach_started_ = now;
      RCLCPP_WARN(get_logger(),
                  "NDT estimates are being rejected (skipping_publish_num=%d >= %d). "
                  "Recovery will trigger if this persists for %.1f s.",
                  skipping, threshold_, hold_sec_);
      return;
    }

    if ((now - breach_started_).seconds() < hold_sec_) return;

    if (last_trigger_.nanoseconds() != 0 &&
        (now - last_trigger_).seconds() < cooldown_sec_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "NDT is still locked (skipping=%d), but recovery is cooling down (%.0f s remaining).",
          skipping, cooldown_sec_ - (now - last_trigger_).seconds());
      return;
    }

    trigger(skipping, now);
  }

  void trigger(int skipping, const rclcpp::Time & now)
  {
    if (!last_pose_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Recovery is required, but no pose has been received on '%s'.",
                            ekf_pose_topic_.c_str());
      return;
    }
    if (!client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Recovery is required, but service '%s' is not ready.",
                            initialize_service_.c_str());
      return;
    }

    auto seed = *last_pose_;
    seed.header.stamp = now;

    // Align'in arama bolgesi bu kovaryanstan tureniyor. EKF'in kendi
    // (fazla iyimser) kovaryansini ezip acikca genis bir bolge veriyoruz.
    std::fill(seed.pose.covariance.begin(), seed.pose.covariance.end(), 0.0);
    seed.pose.covariance[0]  = search_stddev_m_ * search_stddev_m_;    // x
    seed.pose.covariance[7]  = search_stddev_m_ * search_stddev_m_;    // y
    seed.pose.covariance[14] = 0.01;                                   // z
    seed.pose.covariance[21] = 0.01;                                   // roll
    seed.pose.covariance[28] = 0.01;                                   // pitch
    seed.pose.covariance[35] = search_stddev_yaw_ * search_stddev_yaw_;  // yaw

    auto request = std::make_shared<InitializeService::Request>();
    request->pose_with_covariance.push_back(seed);
    request->method = static_cast<uint8_t>(initialize_method_);

    RCLCPP_ERROR(get_logger(),
                 "NDT has been locked for %.1f s (skipping_publish_num=%d). "
                 "RECOVERY: realigning within %.1f m of (%.2f, %.2f). "
                 "Localization will be unavailable during alignment.",
                 (now - breach_started_).seconds(), skipping,
                 search_stddev_m_, seed.pose.pose.position.x, seed.pose.pose.position.y);

    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "  (enabled:=false: service was not called; report only)");
      last_trigger_ = now;
      breach_started_ = rclcpp::Time(0, 0, now.get_clock_type());
      return;
    }

    client_->async_send_request(
        request,
        [this](rclcpp::Client<InitializeService>::SharedFuture future) {
          try {
            const auto response = future.get();
            if (response->status.success) {
              RCLCPP_INFO(get_logger(), "Recovery succeeded; NDT was realigned.");
            } else {
              RCLCPP_ERROR(get_logger(), "Recovery FAILED (code=%u): %s",
                           response->status.code, response->status.message.c_str());
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Recovery service failed: %s", e.what());
          }
        });

    last_trigger_ = now;
    // Align bittikten sonra sayac dogal olarak dusecek; yine de bekleme
    // penceresini sifirla ki cooldown biter bitmez tekrar tetiklenmesin.
    breach_started_ = rclcpp::Time(0, 0, now.get_clock_type());
  }

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Client<InitializeService>::SharedPtr client_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;

  rclcpp::Time breach_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trigger_{0, 0, RCL_ROS_TIME};

  bool enabled_;
  int threshold_;
  int initialize_method_;
  double hold_sec_;
  double cooldown_sec_;
  double search_stddev_m_;
  double search_stddev_yaw_;
  std::string status_name_;
  std::string key_name_;
  std::string ekf_pose_topic_;
  std::string initialize_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtRecoveryWatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
