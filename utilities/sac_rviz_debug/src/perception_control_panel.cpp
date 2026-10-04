#include "sac_rviz_debug/perception_control_panel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <QColor>
#include <QFrame>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QPainter>
#include <QScrollArea>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sac_interfaces/msg/camera_detection_array.hpp>
#include <sac_interfaces/msg/detected_object_array.hpp>
#include <sac_interfaces/msg/road_hazard_array.hpp>
#include <sac_interfaces/msg/tracked_object_array.hpp>

namespace sac_rviz_debug
{
namespace
{
constexpr double kSensorStaleSec = 1.0;
constexpr double kCommandStaleSec = 0.5;
constexpr double kConstraintStaleSec = 0.5;
const QColor kGreen(16, 100, 38);
const QColor kAmber(145, 90, 0);
const QColor kRed(165, 28, 28);
const QColor kGray(95, 95, 95);

QString localizedReason(const std::string & reason)
{
  if (reason == "camera unavailable or stale") return "kamera verisi yok veya bayat";
  if (reason == "map speed limit") return "harita hız sınırı";
  if (reason == "missing/stale/invalid map speed limit") return "harita hız sınırı yok, bayat veya geçersiz";
  if (reason == "red light until stable green") return "kararlı yeşil ışığa kadar kırmızı ışık duruşu";
  if (reason == "hazard observations unavailable or stale") return "yol tehlikesi verisi yok veya bayat";
  if (reason == "path unavailable or stale") return "yol verisi yok veya bayat";
  if (reason == "vehicle transform stale") return "araç dönüşümü bayat";
  if (reason == "road hazard approach/crossing") return "yol tehlikesine yaklaşma/geçiş";
  if (reason == "clear") return "açık";
  if (reason.rfind("STOP ", 0) == 0) return "DUR levhası: " + QString::fromStdString(reason.substr(5));
  return QString::fromStdString(reason);
}

QTableWidget * makeTable(int rows, const QStringList & headings, QWidget * parent)
{
  auto * table = new QTableWidget(rows, headings.size(), parent);
  table->setHorizontalHeaderLabels(headings);
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  for (int col = 1; col < headings.size(); ++col) {
    table->horizontalHeader()->setSectionResizeMode(col, QHeaderView::ResizeToContents);
  }
  table->verticalHeader()->hide();
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionMode(QAbstractItemView::NoSelection);
  table->setAlternatingRowColors(true);
  table->setMinimumHeight(table->horizontalHeader()->height() +
                          rows * table->verticalHeader()->defaultSectionSize() + 8);
  return table;
}

QGroupBox * group(const QString & title, QWidget * widget, QWidget * parent)
{
  auto * box = new QGroupBox(title, parent);
  auto * layout = new QVBoxLayout(box);
  layout->setContentsMargins(4, 6, 4, 4);
  layout->addWidget(widget);
  return box;
}
}  // namespace

SteeringWidget::SteeringWidget(QWidget * parent) : QWidget(parent)
{
  setMinimumSize(132, 132);
  setToolTip("Direksiyon komut açısı; çizim komut derecesi kadar döner.");
}

void SteeringWidget::setAngle(double angle_rad, bool fresh)
{
  angle_rad_ = angle_rad;
  fresh_ = fresh;
  update();
}

void SteeringWidget::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const auto color = fresh_ ? QColor(28, 116, 55) : QColor(130, 130, 130);
  const double cx = width() / 2.0;
  const double cy = height() / 2.0 - 9.0;
  const double radius = std::min(width(), height()) * 0.34;
  p.translate(cx, cy);
  p.setPen(QPen(color, 5, Qt::SolidLine, Qt::RoundCap));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(QPointF(0, 0), radius, radius);
  p.rotate(angle_rad_ * 180.0 / M_PI);
  p.setPen(QPen(color, 5, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(QPointF(0, -radius + 3), QPointF(0, -10));
  p.drawLine(QPointF(-radius * 0.84, radius * 0.54), QPointF(-8, 6));
  p.drawLine(QPointF(radius * 0.84, radius * 0.54), QPointF(8, 6));
  p.setBrush(color);
  p.drawEllipse(QPointF(0, 0), 10, 10);
  p.resetTransform();
  p.setPen(color);
  p.drawText(QRectF(0, height() - 22, width(), 20), Qt::AlignCenter,
             fresh_ ? QString("%1°").arg(angle_rad_ * 180.0 / M_PI, 0, 'f', 1) : "—°");
}

PerceptionControlPanel::PerceptionControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  sensors_ = {{
    {"/velodyne_points"},
    {"/zed/zed_node/imu/data"},
    {"/zed/zed_node/left/image_rect_color"},
    {"/zed/zed_node/left/camera_info"},
    {"/zed/zed_node/odom"},
    {"/encoder_speed"}
  }};
  constraints_[0].label = "Harita";
  constraints_[0].topic = "/planning/map/speed_constraint";
  constraints_[1].label = "Trafik";
  constraints_[1].topic = "/planning/traffic/speed_constraint";
  constraints_[2].label = "Yol tehlikesi";
  constraints_[2].topic = "/planning/road_hazard/speed_constraint";
  constraints_[3].label = "Dinamik nesne";
  constraints_[3].topic = "/planning/dynamic/speed_constraint";
  commands_[0].label = "Denetleyici";
  commands_[0].topic = "/control/controller_command";
  commands_[1].label = "Boylamsal";
  commands_[1].topic = "/control/longitudinal_command";
  commands_[2].label = "Guardian";
  commands_[2].topic = "/vehicle/actuator_command";
  inputs_ = {{
    {"Levha tespitleri", "/yolo_detections"},
    {"Metrik nesneler", "/perception/dynamic_objects"},
    {"Takipli nesneler", "/perception/tracked_objects"},
    {"Yol tehlikeleri", "/perception/road_hazards"},
    {"Yörünge noktaları", "/trajectory_planner/trajectory"}
  }};
  buildUi();
}

void PerceptionControlPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(5, 5, 5, 5);
  auto * dashboard = new QWidget(this);
  auto * dashboard_layout = new QHBoxLayout(dashboard);
  dashboard_layout->setContentsMargins(2, 2, 2, 2);
  auto speedCard = [this](const QString & name, QLabel *& value) {
    auto * card = new QFrame(this);
    card->setFrameShape(QFrame::StyledPanel);
    auto * layout = new QVBoxLayout(card);
    auto * heading = new QLabel(name, card);
    heading->setStyleSheet("font-weight: bold; color: #666;");
    value = new QLabel("— m/s", card);
    value->setStyleSheet("font-size: 22pt; font-weight: bold; color: #888;");
    layout->addWidget(heading);
    layout->addWidget(value);
    layout->addStretch();
    return card;
  };
  auto * speeds = new QWidget(dashboard);
  auto * speeds_layout = new QVBoxLayout(speeds);
  speeds_layout->setContentsMargins(0, 0, 0, 0);
  speeds_layout->addWidget(speedCard("HEDEF HIZ · Controller", target_speed_label_));
  speeds_layout->addWidget(speedCard("ÖLÇÜLEN HIZ · encoder", measured_speed_label_));
  dashboard_layout->addWidget(speeds, 1);
  auto * steering_card = new QFrame(dashboard);
  steering_card->setFrameShape(QFrame::StyledPanel);
  auto * steering_layout = new QVBoxLayout(steering_card);
  auto * steering_heading = new QLabel("DİREKSİYON KOMUTU", steering_card);
  steering_heading->setStyleSheet("font-weight: bold; color: #666;");
  steering_widget_ = new SteeringWidget(steering_card);
  steering_layout->addWidget(steering_heading);
  steering_layout->addWidget(steering_widget_);
  dashboard_layout->addWidget(steering_card);
  root->addWidget(dashboard);
  status_label_ = new QLabel("Komut durumu: veri bekleniyor", this);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("font-weight: bold; color: #a51c1c;");
  root->addWidget(status_label_);

  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * details = new QWidget(scroll);
  auto * detail_layout = new QVBoxLayout(details);
  detail_layout->setContentsMargins(0, 0, 0, 0);

  sensor_table_ = makeTable(6, {"Sensör topic", "Hz (2 s)", "Yaş (s)", "Durum"}, this);
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    setCell(sensor_table_, i, 0, QString::fromStdString(sensors_[i].topic));
  }
  detail_layout->addWidget(group("Sensörler", sensor_table_, details));

  input_table_ = makeTable(5, {"Algı / yol", "Sayı", "Yaş (s)"}, this);
  for (std::size_t i = 0; i < inputs_.size(); ++i) {
    setCell(input_table_, i, 0, QString::fromStdString(inputs_[i].label));
    input_table_->item(i, 0)->setToolTip(QString::fromStdString(inputs_[i].topic));
  }
  detail_layout->addWidget(group("Algı ve yörünge", input_table_, details));

  constraint_table_ = makeTable(4, {"Kısıt", "Azami m/s", "Yaş (s)", "Gerekçe"}, this);
  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    setCell(constraint_table_, i, 0, QString::fromStdString(constraints_[i].label));
    constraint_table_->item(i, 0)->setToolTip(QString::fromStdString(constraints_[i].topic));
  }
  detail_layout->addWidget(group("Hız kısıtları", constraint_table_, details));

  behavior_label_ = new QLabel("Davranış: mesaj yok", this);
  guardian_label_ = new QLabel("Guardian: mesaj yok", this);
  switch_label_ = new QLabel("Lattice geçişi: mesaj yok", this);
  for (auto * label : {behavior_label_, guardian_label_, switch_label_}) {
    label->setWordWrap(true);
    detail_layout->addWidget(label);
  }

  command_table_ = makeTable(3,
    {"Komut", "Hedef m/s", "Direksiyon °", "Gaz", "Fren", "Yaş (s)", "Mod"}, this);
  for (std::size_t i = 0; i < commands_.size(); ++i) {
    setCell(command_table_, i, 0, QString::fromStdString(commands_[i].label));
    command_table_->item(i, 0)->setToolTip(QString::fromStdString(commands_[i].topic));
  }
  detail_layout->addWidget(group("Controller → boylamsal → Guardian", command_table_, details));

  note_label_ = new QLabel("—", this);
  note_label_->setWordWrap(true);
  detail_layout->addWidget(note_label_);
  detail_layout->addStretch();
  scroll->setWidget(details);
  root->addWidget(scroll, 1);
}

void PerceptionControlPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  const auto suffix = std::to_string(
    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
  node_ = std::make_shared<rclcpp::Node>("sac_rviz_debug_" + suffix);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  subscribeTopics();
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    // All callbacks update small metadata only. The RViz process does not share the control executor.
    executor_->spin_some(std::chrono::milliseconds(5));
    refresh();
  });
  timer_->start(200);
}

void PerceptionControlPanel::subscribeTopics()
{
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  const auto latest_qos = rclcpp::QoS(1).reliable().durability_volatile();
  auto add = [this](auto subscription) {subscriptions_.push_back(subscription);};
  add(node_->create_subscription<sensor_msgs::msg::PointCloud2>(sensors_[0].topic, sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {recordSensor(0);}));
  add(node_->create_subscription<sensor_msgs::msg::Imu>(sensors_[1].topic, sensor_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr) {recordSensor(1);}));
  add(node_->create_subscription<sensor_msgs::msg::Image>(sensors_[2].topic, sensor_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr) {recordSensor(2);}));
  add(node_->create_subscription<sensor_msgs::msg::CameraInfo>(sensors_[3].topic, sensor_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {recordSensor(3);}));
  add(node_->create_subscription<nav_msgs::msg::Odometry>(sensors_[4].topic, sensor_qos,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr) {recordSensor(4);}));
  add(node_->create_subscription<std_msgs::msg::Float32>(sensors_[5].topic, sensor_qos,
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
      encoder_speed_mps_ = msg->data;
      recordSensor(5);
    }));

  add(node_->create_subscription<sac_interfaces::msg::CameraDetectionArray>(inputs_[0].topic,
    sensor_qos, [this](sac_interfaces::msg::CameraDetectionArray::ConstSharedPtr msg) {
      inputs_[0].count = msg->detections.size(); inputs_[0].last = Clock::now(); inputs_[0].seen = true;
      sign_stamp_ns_ = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        msg->header.stamp.nanosec;
    }));
  add(node_->create_subscription<sac_interfaces::msg::DetectedObjectArray>(inputs_[1].topic,
    sensor_qos, [this](sac_interfaces::msg::DetectedObjectArray::ConstSharedPtr msg) {
      inputs_[1].count = msg->objects.size(); inputs_[1].last = Clock::now(); inputs_[1].seen = true;
    }));
  add(node_->create_subscription<sac_interfaces::msg::TrackedObjectArray>(inputs_[2].topic,
    sensor_qos, [this](sac_interfaces::msg::TrackedObjectArray::ConstSharedPtr msg) {
      inputs_[2].count = msg->objects.size(); inputs_[2].last = Clock::now(); inputs_[2].seen = true;
    }));
  add(node_->create_subscription<sac_interfaces::msg::RoadHazardArray>(inputs_[3].topic,
    sensor_qos, [this](sac_interfaces::msg::RoadHazardArray::ConstSharedPtr msg) {
      inputs_[3].count = msg->hazards.size(); inputs_[3].last = Clock::now(); inputs_[3].seen = true;
    }));
  add(node_->create_subscription<nav_msgs::msg::Path>(inputs_[4].topic,
    sensor_qos, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
      inputs_[4].count = msg->poses.size(); inputs_[4].last = Clock::now(); inputs_[4].seen = true;
    }));

  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    add(node_->create_subscription<sac_interfaces::msg::SpeedConstraint>(constraints_[i].topic,
      latest_qos, [this, i](sac_interfaces::msg::SpeedConstraint::ConstSharedPtr msg) {
        constraints_[i].msg = *msg; constraints_[i].last = Clock::now(); constraints_[i].seen = true;
      }));
  }
  for (std::size_t i = 0; i < commands_.size(); ++i) {
    add(node_->create_subscription<sac_interfaces::msg::ActuatorCommand>(commands_[i].topic,
      latest_qos, [this, i](sac_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) {
        commands_[i].msg = *msg; commands_[i].last = Clock::now(); commands_[i].seen = true;
      }));
  }
  add(node_->create_subscription<sac_interfaces::msg::BehaviorStatus>(
    "/planning/behavior/status", latest_qos,
    [this](sac_interfaces::msg::BehaviorStatus::ConstSharedPtr msg) {
      behavior_ = *msg; behavior_last_ = Clock::now(); behavior_seen_ = true;
    }));
  add(node_->create_subscription<sac_interfaces::msg::CommandGuardianStatus>(
    "/safety/command_guardian/status", latest_qos,
    [this](sac_interfaces::msg::CommandGuardianStatus::ConstSharedPtr msg) {
      guardian_ = *msg; guardian_last_ = Clock::now(); guardian_seen_ = true;
    }));
  add(node_->create_subscription<std_msgs::msg::Bool>(
    "/switch_controller", latest_qos,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      switch_active_ = msg->data; switch_last_ = Clock::now(); switch_seen_ = true;
    }));
}

void PerceptionControlPanel::recordSensor(std::size_t index)
{
  auto & state = sensors_[index];
  const auto now = Clock::now();
  state.last = now;
  ++state.count;
  state.arrivals.push_back(now);
  while (!state.arrivals.empty() && ageSeconds(state.arrivals.front(), now) > 2.0) {
    state.arrivals.pop_front();
  }
}

double PerceptionControlPanel::ageSeconds(TimePoint last, TimePoint now)
{
  return std::chrono::duration<double>(now - last).count();
}

QString PerceptionControlPanel::number(double value, int precision)
{
  return std::isfinite(value) ? QString::number(value, 'f', precision) : QString("—");
}

void PerceptionControlPanel::setCell(QTableWidget * table, int row, int col,
                                    const QString & text, const QColor & color)
{
  auto * item = table->item(row, col);
  if (!item) {
    item = new QTableWidgetItem();
    table->setItem(row, col, item);
  }
  item->setText(text);
  if (color.isValid()) item->setForeground(color);
}

void PerceptionControlPanel::refresh()
{
  const auto now = Clock::now();
  const double controller_age = commands_[0].seen ? ageSeconds(commands_[0].last, now) :
    std::numeric_limits<double>::infinity();
  const bool controller_fresh = controller_age <= kCommandStaleSec;
  const double encoder_age = sensors_[5].count ? ageSeconds(sensors_[5].last, now) :
    std::numeric_limits<double>::infinity();
  const bool encoder_fresh = encoder_age <= kSensorStaleSec &&
    std::isfinite(encoder_speed_mps_);
  target_speed_label_->setText(controller_fresh ?
    QString("%1 m/s").arg(number(commands_[0].msg.target_speed_mps)) : "— m/s");
  target_speed_label_->setStyleSheet(controller_fresh ?
    "font-size: 22pt; font-weight: bold; color: #106426;" :
    "font-size: 22pt; font-weight: bold; color: #a51c1c;");
  measured_speed_label_->setText(encoder_fresh ?
    QString("%1 m/s").arg(number(encoder_speed_mps_)) : "— m/s");
  measured_speed_label_->setStyleSheet(encoder_fresh ?
    "font-size: 22pt; font-weight: bold; color: #106426;" :
    "font-size: 22pt; font-weight: bold; color: #a51c1c;");
  steering_widget_->setAngle(controller_fresh &&
    std::isfinite(commands_[0].msg.steering_angle_rad) ?
      commands_[0].msg.steering_angle_rad : 0.0,
    controller_fresh && std::isfinite(commands_[0].msg.steering_angle_rad));
  if (!controller_fresh) {
    status_label_->setText("Komut durumu: denetleyici mesajı yok veya bayat");
    status_label_->setStyleSheet("font-weight: bold; color: #a51c1c;");
  } else if (commands_[0].msg.mode ==
             sac_interfaces::msg::ActuatorCommand::MODE_CONTROLLED_STOP) {
    QString reason = "yol, lokalizasyon, hedef veya hız kısıtı kontrol edilmeli";
    for (const auto & constraint : constraints_) {
      if (constraint.seen && ageSeconds(constraint.last, now) <= kConstraintStaleSec &&
          std::isfinite(constraint.msg.max_speed_mps) &&
          constraint.msg.max_speed_mps <= 0.0F) {
        reason = QString::fromStdString(constraint.label) + ": " +
          localizedReason(constraint.msg.reason);
        break;
      }
    }
    const auto & stamp = commands_[0].msg.source_stamp;
    const int64_t command_stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL +
      stamp.nanosec;
    if (inputs_[0].seen && ageSeconds(inputs_[0].last, now) <= kSensorStaleSec &&
        sign_stamp_ns_ > 0 && command_stamp_ns > 0) {
      const double stamp_gap = std::abs(static_cast<double>(command_stamp_ns - sign_stamp_ns_)) / 1e9;
      if (stamp_gap > 1.0) {
        reason += QString(" | kamera/denetleyici zaman farkı %1 s").arg(number(stamp_gap, 1));
      }
    }
    status_label_->setText("KONTROLLÜ DURUŞ · " + reason);
    status_label_->setStyleSheet("font-weight: bold; color: #a51c1c;");
  } else {
    status_label_->setText("Denetleyici komut modu: " +
      QString::number(commands_[0].msg.mode));
    status_label_->setStyleSheet("font-weight: bold; color: #106426;");
  }
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    auto & state = sensors_[i];
    while (!state.arrivals.empty() && ageSeconds(state.arrivals.front(), now) > 2.0) {
      state.arrivals.pop_front();
    }
    const double age = state.count ? ageSeconds(state.last, now) :
      std::numeric_limits<double>::infinity();
    const double hz = state.arrivals.size() >= 2 ?
      (state.arrivals.size() - 1) / ageSeconds(state.arrivals.front(), state.arrivals.back()) : 0.0;
    const bool fresh = age <= kSensorStaleSec;
    setCell(sensor_table_, i, 1, number(hz, 1), fresh ? kGreen : kGray);
    setCell(sensor_table_, i, 2, number(age), fresh ? kGreen : kRed);
    setCell(sensor_table_, i, 3, fresh ? "TAMAM" : (state.count ? "BAYAT" : "VERİ YOK"),
            fresh ? kGreen : kRed);
  }
  sensor_table_->item(5, 0)->setText(
    QString::fromStdString(sensors_[5].topic) +
    (sensors_[5].count ? QString(" (%1 m/s)").arg(number(encoder_speed_mps_)) : QString()));

  for (std::size_t i = 0; i < inputs_.size(); ++i) {
    const auto & state = inputs_[i];
    const double age = state.seen ? ageSeconds(state.last, now) :
      std::numeric_limits<double>::infinity();
    setCell(input_table_, i, 1, state.seen ? QString::number(state.count) : "—");
    setCell(input_table_, i, 2, number(age), age <= kSensorStaleSec ? kGreen : kRed);
  }

  double fresh_min = std::numeric_limits<double>::infinity();
  QString min_source;
  for (std::size_t i = 0; i < constraints_.size(); ++i) {
    const auto & state = constraints_[i];
    const double age = state.seen ? ageSeconds(state.last, now) :
      std::numeric_limits<double>::infinity();
    const bool fresh = age <= kConstraintStaleSec &&
      std::isfinite(state.msg.max_speed_mps) && state.msg.max_speed_mps >= 0.0F;
    setCell(constraint_table_, i, 1, state.seen ? number(state.msg.max_speed_mps) : "—",
            fresh ? kGreen : kRed);
    setCell(constraint_table_, i, 2, number(age), fresh ? kGreen : kRed);
    setCell(constraint_table_, i, 3,
            state.seen ? localizedReason(state.msg.reason) : "—");
    if (fresh && state.msg.max_speed_mps < fresh_min) {
      fresh_min = state.msg.max_speed_mps;
      min_source = QString::fromStdString(state.label);
    }
  }

  const double behavior_age = behavior_seen_ ? ageSeconds(behavior_last_, now) :
    std::numeric_limits<double>::infinity();
  behavior_label_->setText(behavior_seen_ ?
    QString("Davranış: %1 | %2 | yaş %3 s%4")
      .arg(QString::fromStdString(behavior_.state), QString::fromStdString(behavior_.reason),
           number(behavior_age), behavior_.stop_required ? " | DUR" : "") :
    "Davranış: mesaj yok");
  behavior_label_->setStyleSheet(behavior_age <= kSensorStaleSec ? "color: #106426" : "color: #a51c1c");

  const double switch_age = switch_seen_ ? ageSeconds(switch_last_, now) :
    std::numeric_limits<double>::infinity();
  switch_label_->setText(switch_seen_ ?
    QString("Lattice geçişi: %1 | yaş %2 s").arg(switch_active_ ? "AÇIK" : "KAPALI", number(switch_age)) :
    "Lattice geçişi: mesaj yok");
  switch_label_->setStyleSheet(switch_age <= 2.0 ? "color: #106426" : "color: #a51c1c");

  const double guardian_age = guardian_seen_ ? ageSeconds(guardian_last_, now) :
    std::numeric_limits<double>::infinity();
  guardian_label_->setText(guardian_seen_ ?
    QString("Guardian: durum %1 | sağlıklı %2 | aktarım %3 | yaş %4 s | %5")
      .arg(guardian_.state).arg(guardian_.healthy ? "evet" : "hayır")
      .arg(guardian_.transport_enabled ? "AÇIK" : "KAPALI")
      .arg(number(guardian_age), QString::fromStdString(guardian_.fault_reason)) :
    "Guardian: mesaj yok");
  guardian_label_->setStyleSheet(
    guardian_age <= kSensorStaleSec && guardian_.healthy ? "color: #106426" : "color: #a51c1c");

  for (std::size_t i = 0; i < commands_.size(); ++i) {
    const auto & state = commands_[i];
    const double age = state.seen ? ageSeconds(state.last, now) :
      std::numeric_limits<double>::infinity();
    const bool fresh = age <= kCommandStaleSec;
    setCell(command_table_, i, 1, state.seen ? number(state.msg.target_speed_mps) : "—",
            fresh ? kGreen : kRed);
    setCell(command_table_, i, 2, state.seen ? number(state.msg.steering_angle_rad * 180.0 / M_PI, 1) : "—");
    setCell(command_table_, i, 3, state.seen ? number(state.msg.throttle_normalized) : "—");
    setCell(command_table_, i, 4, state.seen ? number(state.msg.brake_normalized) : "—");
    setCell(command_table_, i, 5, number(age), fresh ? kGreen : kRed);
    setCell(command_table_, i, 6, state.seen ?
      QString("%1 / %2").arg(state.msg.mode).arg(state.msg.enable ? "etkin" : "kapalı") : "—");
  }
  note_label_->setText(std::isfinite(fresh_min) ?
    QString("Son alınan en düşük kısıt: %1 m/s (%2). Denetleyici kaynak zamanını ve yol eğriliğini de denetler.")
      .arg(number(fresh_min), min_source) :
    "Güncel hız kısıtı alınmadı. Yayıncıları ve denetleyici ayarlarını kontrol edin.");
  note_label_->setStyleSheet(std::isfinite(fresh_min) ? "color: #916000" : "color: #a51c1c");
}
}  // namespace sac_rviz_debug

PLUGINLIB_EXPORT_CLASS(sac_rviz_debug::PerceptionControlPanel, rviz_common::Panel)
