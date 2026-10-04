#ifndef _ESKF_HPP_
#define _ESKF_HPP_

#include <chrono>
#include <memory>

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>

#include <yaml-cpp/yaml.h>

#include "ESKF/type.hpp"
struct GnssFix {
  Eigen::Vector3d lla;           // lat,lon,alt [rad,rad,m]
  std::chrono::steady_clock::time_point           stamp;
};
class ErrorStateKalmanFilter {
 public:
  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
  using Mat15d = Eigen::Matrix<double, 15, 15>;
  using Mat12d = Eigen::Matrix<double, 12, 12>;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Vec15d = Eigen::Matrix<double, 15, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;


  double g_ = -9.81; // Assuming frame is ENU

  ErrorStateKalmanFilter(const YAML::Node& imuCalibration);
  ~ErrorStateKalmanFilter() {}

  void predictWithImu(std::shared_ptr<ImuMeasurement> imuData);
  void updateWithGnss(const Eigen::Vector3d& llaPosition);
  void updateWithWheelEncoder(const double& encoderData);
  void updateWithMagYaw(const double& mag_yaw_rad);
  void updateWithGnssYaw(const double& course_rad);


  double computeCourseFromGnss(const GnssFix& curr);

  Eigen::Vector3d llaToEnu(const Eigen::Vector3d& llaPosition);
  Eigen::Vector3d enuToLla(const Eigen::Vector3d& enuPosition);

  void printState() const;
  Eigen::Vector3d getPosition() const { return nominal_pos_; }
  Eigen::Vector3d getVelocity() const { return nominal_vel_; }
  Eigen::Quaterniond getQuaternion() const { return nominal_attitude_; }

 private:
  void injectErrorToNominal();
  void resetErrorState();

  Eigen::Matrix<double, 4, 3> computeQuatJacobiToErrorQuat();
  Eigen::Matrix<double, 6, 1> computeHx();
  Eigen::Matrix<double, 1, 3> computeWheelEncoderJacobian();
  Eigen::Matrix<double, 1, 3> computeMagYawJacobian() const;

  TimePoint prev_time_;
  bool is_mag_yaw_initialized_ = false;

  Eigen::Vector3d nominal_pos_;
  Eigen::Vector3d nominal_vel_;
  Eigen::Quaterniond nominal_attitude_;
  Eigen::Vector3d nominal_accel_bias_;
  Eigen::Vector3d nominal_gyro_bias_;
  Eigen::Vector3d gravity_vector_{0.0, 0.0, g_};              // (0,0,-9.81)

  Vec15d error_x_;
  Mat15d P_;

  Mat15d F_x_;
  Eigen::Matrix<double, 15, 12> F_i_;
  Mat12d Q_i_;

  Vec6d z_;
  Eigen::Matrix<double, 1, 1> z_encoder_;
  Eigen::Matrix<double, 6, 16> H_x_;
  Eigen::Matrix<double, 1, 16> H_x_encoder_;
  Eigen::Matrix<double, 1, 16> H_x_magyaw_{Eigen::Matrix<double,1,16>::Zero()};

  Eigen::Matrix<double, 16, 15> J_true_error_;
  Eigen::Matrix<double, 6, 15> H_;
  Eigen::Matrix<double, 1, 15> H_encoder_;
  Eigen::Matrix<double, 1, 15> H_magyaw_;     // = H_x_magyaw_ * J_true_error_
  
  // Measurement Noise covariance matrices
  Mat6d V_;
  Eigen::Matrix<double, 1, 1> V_encoder_;
  Eigen::Matrix<double, 1, 1>  V_magyaw_{Eigen::Matrix<double,1,1>::Identity()};

  // Gains placeholders
  Eigen::Matrix<double, 15, 6> K_;
  Eigen::Matrix<double, 15, 1> K_encoder_;
  Eigen::Matrix<double, 15, 1> K_magyaw_;




  std::chrono::steady_clock::time_point prev_fix_time_{};
  Eigen::Vector3d                        prev_fix_enu_{Eigen::Vector3d::Zero()};
  bool                                   have_prev_fix_{false};

  // helper
  Mat15d G_;

  bool local_cartesian_initialized_ = false;
  GeographicLib::LocalCartesian local_cartesian_;
};

#endif  // _ESKF_HPP_