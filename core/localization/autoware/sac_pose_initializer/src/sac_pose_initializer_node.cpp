#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware/component_interface_specs/localization.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Autoware Core 1.9 arayuzu:
//   autoware_localization_msgs/srv/InitializeLocalization
// (Eski vendor surumu tier4_localization_msgs kullaniyordu; ayni servis
//  adi fakat farkli tip oldugu icin istemci sunucuya baglanamiyordu.)
using InitializeService =
    autoware::component_interface_specs::localization::Initialize::Service;

/**
 * RViz'den (veya baska bir kaynaktan) gelen ilk pozu harita frame'ine
 * cevirir, z degerini HARITADAN okur ve lokalizasyon yigininin initialize
 * servisini cagirir.
 *
 * Neden z'yi burada hesapliyoruz:
 *   RViz'in "2D Pose Estimate" araci z=0 yayinlar ve kovaryansin sadece
 *   x (cov[0]), y (cov[7]) ve yaw (cov[35]) elemanlarini doldurur.
 *   cov[14] (z) SIFIR kalir. NDT'nin ndt_align aramasi ornekleme
 *   sigmalarini bu matristen kurdugu icin z sigmasi 0 olur ve z HIC
 *   ARANMAZ. Yani yanlis bir z, align'in telafi edemeyecegi sabit bir
 *   hatadir: arac harita zemininin altinda kalir, cevresinde eslesecek
 *   nokta bulunmaz ve NDT hicbir zaman converge olmaz.
 *
 *   Bu dosya eskiden z'yi tek bir sabitle (map_z) eziyordu. Bu harita icin
 *   olculen sonuc: guzergah 22.9 m tirmaniyor (LIO-SAM yorunge z araligi
 *   -1.80 .. 21.11 m), dolayisiyla map_z=0.0 ile medyan hata 3.28 m,
 *   p95 20.04 m ve yolun %62'si 2 m'den fazla sapiyordu. 2 m,
 *   resolution 2.0'da NDT'nin voxel arama yaricapidir.
 *
 * Neden autoware_map_height_fitter kullanilmiyor:
 *   Dogru mimari o olurdu ve denendi. Bu 1.9.0 ikililerinde iki sorun var:
 *     1) pose_initializer icindeki fitter YAPILANDIRILAMIYOR.
 *        map_height_fitter.target / .map_loader_name parametreleri o
 *        dugumde declare EDILMIYOR; params dosyasindan da CLI'dan da
 *        gecmiyor, dugum "Failed to get parameters" uyarisi basip atil
 *        kaliyor ve initialize servisi buna ragmen success=True donuyor
 *        (sessiz basarisizlik).
 *     2) Standalone autoware_map_height_fitter_node calisiyor ve servisini
 *        aciyor, ama bu haritada olculen basari orani 7 denemede 2.
 *        Basardiginda dogru (hata < 20 cm), fakat 100 m ve 25 m karo
 *        boyutlarinin ikisinde de ayni noktalarda basarisiz oluyor.
 *   Guvenlik acisindan kritik ve seyrek calisan bir yolda %70 basarisiz
 *   olan, ustelik merkezi olarak yapilandirilamayan bir bilesene
 *   guvenilemez. Yaptigi is (x,y civarindaki zemin yuksekligi) bizim
 *   zaten abone oldugumuz veri uzerinde basit aritmetik.
 *
 * Tum topic / servis / frame isimleri parametre dosyasindan gelir.
 */
class SacPoseInitializerNode : public rclcpp::Node
{
public:
  SacPoseInitializerNode()
      : Node("sac_pose_initializer"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    // "map": z haritadan okunur (varsayilan). "constant": map_z kullanilir.
    height_source_ = this->declare_parameter<std::string>("height_source", "map");
    // Zemin izgarasinin kuruldugu harita bulutu. Tam harita en dogru sonucu
    // verir ve latched (transient_local) oldugu icin bir kez gelir.
    // Bellek darsa /map/downsampled_pointcloud_map kullanilabilir.
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map/pointcloud_map");
    // Zemin yuksekligi bu yaricaptaki hucrelerin medyani olarak alinir.
    ground_search_radius_ = this->declare_parameter<double>("ground_search_radius", 5.0);
    ground_grid_size_ = this->declare_parameter<double>("ground_grid_size", 1.0);
    // base_link zemin hizasinda kabul edilir. Farkli bir referans icin.
    z_offset_ = this->declare_parameter<double>("z_offset", 0.0);
    // Harita gelmediginde / o noktada nokta olmadiginda kullanilan yedek.
    map_z_ = this->declare_parameter<double>("map_z", 0.0);
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    initial_pose_topic_ = this->declare_parameter<std::string>("initial_pose_topic", "initialpose");
    initialize_service_ =
        this->declare_parameter<std::string>("initialize_service", "/localization/initialize");
    // 0=AUTO: verilen yaklasik pozu once NDT align ile haritaya oturtur.
    // 1=DIRECT: pozu hizalama yapmadan dogrudan EKF/NDT'ye resetler.
    initialize_method_ = this->declare_parameter<int>("initialize_method", 0);
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout", 0.5);

    RCLCPP_INFO(this->get_logger(), "sac_pose_initializer started");
    RCLCPP_INFO(this->get_logger(), "  initial_pose_topic: %s", initial_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  initialize_service: %s", initialize_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  height_source: %s", height_source_.c_str());
    RCLCPP_INFO(this->get_logger(), "  map_z (yedek): %.2f", map_z_);

    if (height_source_ == "map") {
      // Harita latched yayinlanir; transient_local olmadan gec baglanan
      // abone hicbir sey almaz.
      rclcpp::QoS map_qos(1);
      map_qos.transient_local().reliable();
      map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          map_topic_, map_qos,
          std::bind(&SacPoseInitializerNode::mapCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "  map_topic: %s (zemin izgarasi bekleniyor)",
                  map_topic_.c_str());
    } else if (height_source_ != "constant") {
      RCLCPP_WARN(this->get_logger(),
                  "height_source '%s' gecersiz; 'map' veya 'constant' olmali. "
                  "'constant' varsayiliyor.", height_source_.c_str());
      height_source_ = "constant";
    }

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, 10,
        std::bind(&SacPoseInitializerNode::initialPoseCallback, this, std::placeholders::_1));

    client_ = this->create_client<InitializeService>(initialize_service_);
  }

private:
  // ---------------------------------------------------------------- //
  // Zemin yuksekligi izgarasi
  //
  // Harita bir kez gelir (latched). Noktalari saklamak yerine 2D hucre
  // basina MINIMUM z tutuyoruz: bu hucredeki en alt yuzey, yani zemin.
  // 10.9M noktalik harita 1 m hucrede ~500k girise inar; bulut hemen
  // serbest kalir.
  // ---------------------------------------------------------------- //
  static std::int64_t cellKey(std::int32_t cx, std::int32_t cy)
  {
    return (static_cast<std::int64_t>(cx) << 32) |
           static_cast<std::uint32_t>(cy);
  }

  std::int32_t toCell(double v) const
  {
    return static_cast<std::int32_t>(std::floor(v / ground_grid_size_));
  }

  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto & f : msg->fields) {
      if (f.name == "x") off_x = static_cast<int>(f.offset);
      else if (f.name == "y") off_y = static_cast<int>(f.offset);
      else if (f.name == "z") off_z = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Harita bulutunda x/y/z alani yok; zemin izgarasi kurulamadi.");
      return;
    }

    const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
    std::unordered_map<std::int64_t, float> grid;
    grid.reserve(n / 16 + 1024);

    std::size_t used = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint8_t * p = msg->data.data() + i * msg->point_step;
      float x, y, z;
      std::memcpy(&x, p + off_x, sizeof(float));
      std::memcpy(&y, p + off_y, sizeof(float));
      std::memcpy(&z, p + off_z, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const auto key = cellKey(toCell(x), toCell(y));
      const auto it = grid.find(key);
      if (it == grid.end()) grid.emplace(key, z);
      else if (z < it->second) it->second = z;
      ++used;
    }

    ground_grid_ = std::move(grid);
    RCLCPP_INFO(this->get_logger(),
                "Zemin izgarasi hazir: %zu nokta -> %zu hucre (%.2f m), topic %s",
                used, ground_grid_.size(), ground_grid_size_, map_topic_.c_str());
  }

  /// (x, y) cevresindeki hucre minimumlarinin MEDYANI.
  /// Medyan, haritadaki tek tuk asagi kacak noktalara karsi dayanikli.
  std::optional<double> groundHeight(double x, double y) const
  {
    if (ground_grid_.empty()) return std::nullopt;

    const int r = static_cast<int>(std::ceil(ground_search_radius_ / ground_grid_size_));
    const std::int32_t cx = toCell(x);
    const std::int32_t cy = toCell(y);

    std::vector<float> zs;
    zs.reserve(static_cast<std::size_t>((2 * r + 1) * (2 * r + 1)));
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (dx * dx + dy * dy > r * r) continue;
        const auto it = ground_grid_.find(
            cellKey(cx + static_cast<std::int32_t>(dx), cy + static_cast<std::int32_t>(dy)));
        if (it != ground_grid_.end()) zs.push_back(it->second);
      }
    }
    if (zs.empty()) return std::nullopt;

    const std::size_t mid = zs.size() / 2;
    std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
    return static_cast<double>(zs[mid]);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!client_->service_is_ready())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Initialize service '%s' is not available yet.", initialize_service_.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_in_source_frame;
    pose_in_source_frame.header = msg->header;
    pose_in_source_frame.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped pose_in_map_frame;

    try
    {
      const auto transform_stamped = tf_buffer_.lookupTransform(
          map_frame_, pose_in_source_frame.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(pose_in_source_frame, pose_in_map_frame, transform_stamped);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform initial pose: %s", ex.what());
      return;
    }

    const double x = pose_in_map_frame.pose.position.x;
    const double y = pose_in_map_frame.pose.position.y;

    double z = map_z_;
    const char * source = "map_z (sabit)";
    if (height_source_ == "map") {
      if (const auto ground = groundHeight(x, y)) {
        z = *ground + z_offset_;
        source = "harita zemini";
      } else if (ground_grid_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Harita bulutu '%s' henuz gelmedi; z icin map_z=%.2f kullaniliyor. "
                    "Yanlissa NDT converge OLMAZ (z aranmaz), pozu harita yuklendikten "
                    "sonra tekrar verin.", map_topic_.c_str(), map_z_);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "(%.2f, %.2f) cevresinde %.1f m icinde harita noktasi yok; "
                    "z icin map_z=%.2f kullaniliyor. Bu poz haritanin DISINDA olabilir.",
                    x, y, ground_search_radius_, map_z_);
      }
    }
    pose_in_map_frame.pose.position.z = z;

    RCLCPP_INFO(this->get_logger(),
                "Ilk poz -> x %.2f  y %.2f  z %.2f  (z kaynagi: %s)", x, y, z, source);

    auto request = std::make_shared<InitializeService::Request>();
    msg->pose.pose = pose_in_map_frame.pose;
    msg->header.frame_id = map_frame_;
    request->pose_with_covariance.push_back(*msg);
    request->method = static_cast<uint8_t>(initialize_method_);

    client_->async_send_request(request);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<InitializeService>::SharedPtr client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  std::unordered_map<std::int64_t, float> ground_grid_;

  std::string height_source_;
  std::string map_topic_;
  double ground_search_radius_;
  double ground_grid_size_;
  double z_offset_;
  double map_z_;
  double tf_timeout_sec_;
  int initialize_method_;
  std::string map_frame_;
  std::string initial_pose_topic_;
  std::string initialize_service_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SacPoseInitializerNode>());
  rclcpp::shutdown();
  return 0;
}
