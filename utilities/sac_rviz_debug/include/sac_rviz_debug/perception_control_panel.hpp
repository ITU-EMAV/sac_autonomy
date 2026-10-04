#pragma once

#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sac_interfaces/msg/actuator_command.hpp>
#include <sac_interfaces/msg/behavior_status.hpp>
#include <sac_interfaces/msg/command_guardian_status.hpp>
#include <sac_interfaces/msg/speed_constraint.hpp>

namespace sac_rviz_debug
{
class SteeringWidget : public QWidget
{
public:
  explicit SteeringWidget(QWidget * parent = nullptr);
  void setAngle(double angle_rad, bool fresh);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  double angle_rad_{0.0};
  bool fresh_{false};
};

class PerceptionControlPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PerceptionControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  struct SensorState {
    std::string topic;
    std::deque<TimePoint> arrivals;
    TimePoint last{};
    uint64_t count{0};
  };
  struct ConstraintState {
    std::string label;
    std::string topic;
    sac_interfaces::msg::SpeedConstraint msg;
    TimePoint last{};
    bool seen{false};
  };
  struct CommandState {
    std::string label;
    std::string topic;
    sac_interfaces::msg::ActuatorCommand msg;
    TimePoint last{};
    bool seen{false};
  };
  struct InputState {
    std::string label;
    std::string topic;
    std::size_t count{0};
    TimePoint last{};
    bool seen{false};
  };

  void buildUi();
  void subscribeTopics();
  void refresh();
  void recordSensor(std::size_t index);
  static double ageSeconds(TimePoint last, TimePoint now);
  static QString number(double value, int precision = 2);
  static void setCell(QTableWidget * table, int row, int col,
                      const QString & text, const QColor & color = QColor());

  std::array<SensorState, 6> sensors_;
  std::array<ConstraintState, 4> constraints_;
  std::array<CommandState, 3> commands_;
  std::array<InputState, 5> inputs_;
  float encoder_speed_mps_{0.0F};
  int64_t sign_stamp_ns_{0};
  sac_interfaces::msg::BehaviorStatus behavior_;
  sac_interfaces::msg::CommandGuardianStatus guardian_;
  TimePoint behavior_last_{};
  TimePoint guardian_last_{};
  bool behavior_seen_{false};
  bool guardian_seen_{false};
  bool switch_seen_{false};
  bool switch_active_{false};
  TimePoint switch_last_{};

  QLabel * behavior_label_{nullptr};
  QLabel * guardian_label_{nullptr};
  QLabel * switch_label_{nullptr};
  QLabel * note_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * target_speed_label_{nullptr};
  QLabel * measured_speed_label_{nullptr};
  SteeringWidget * steering_widget_{nullptr};
  QTableWidget * sensor_table_{nullptr};
  QTableWidget * constraint_table_{nullptr};
  QTableWidget * command_table_{nullptr};
  QTableWidget * input_table_{nullptr};
  QTimer * timer_{nullptr};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};
}  // namespace sac_rviz_debug
