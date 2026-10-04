#include <cmath>
#include <iostream>

#include <spdlog/spdlog.h>

#include "ESKF/eskf.hpp"
#include "ESKF/math_utils.hpp"

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node& imuCalibration)
    : nominal_pos_(Eigen::Vector3d::Zero()),
      nominal_vel_(Eigen::Vector3d::Zero()),
      nominal_attitude_(Eigen::Quaterniond::Identity()),
      nominal_accel_bias_(Eigen::Vector3d::Zero()),
      nominal_gyro_bias_(Eigen::Vector3d::Zero()),
      error_x_(Vec15d::Zero()),
      P_(Mat15d::Identity()),
      F_x_(Mat15d::Identity()),
      F_i_(Eigen::Matrix<double, 15, 12>::Zero()),
      Q_i_(Mat12d::Identity()),
      H_x_(Eigen::Matrix<double, 6, 16>::Zero()),
      H_x_encoder_(Eigen::Matrix<double, 1, 16>::Zero()),
      J_true_error_(Eigen::Matrix<double, 16, 15>::Zero()),
      V_(Mat6d::Identity()),
      G_(Mat15d::Identity()) {
  F_i_.block<12, 12>(3, 0) = Mat12d::Identity();
  H_x_.block<6, 6>(0, 0) = Mat6d::Identity();
  H_x_.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
  J_true_error_.block<6, 6>(0, 0) = Mat6d::Identity();
  J_true_error_.block<6, 6>(10, 9) = Mat6d::Identity();
  J_true_error_.block<4, 3>(6, 6) = computeQuatJacobiToErrorQuat();

  auto accel_noise = imuCalibration["accelerometer_noise"].as<double>();
  auto accel_bias_stability = imuCalibration["accelerometer_bias_stability"].as<double>();
  auto gyro_noise = MathUtils::degreeToRadian(imuCalibration["gyroscope_noise"].as<double>());
  auto gyro_bias_stability = MathUtils::degreeToRadian(imuCalibration["gyroscope_bias_stability"].as<double>());
  auto update_rate = imuCalibration["update_rate"].as<double>();

  Q_i_.block<3, 3>(0, 0) = std::pow(accel_noise, 2) * update_rate * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(3, 3) = std::pow(gyro_noise, 2) * update_rate * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(6, 6) = std::pow(accel_bias_stability * update_rate, 2) * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(9, 9) = std::pow(gyro_bias_stability * update_rate, 2) * Eigen::Matrix3d::Identity();

  // 1) Position uncertainty 10 m
  double sigma_pos  = 5.0;                // meters
  P_.block<3,3>( 0,  0) = Eigen::Matrix3d::Identity() * (sigma_pos * sigma_pos);

  // 2) Velocity uncertainty 1 m/s
  double sigma_vel  = 1.0;                // m/s
  P_.block<3,3>( 3,  3) = Eigen::Matrix3d::Identity() * (sigma_vel * sigma_vel);

  // 3) Attitude-error uncertainty 30 degre
  double sigma_att  = 30.0 * M_PI/180.0;   // radians
  P_.block<3,3>( 6,  6) = Eigen::Matrix3d::Identity() * (sigma_att * sigma_att);

  // 4) Accelerometer-bias uncertainty 1.0 m/s²
  double sigma_abias = 1.0;               // m/s²
  P_.block<3,3>( 9,  9) = Eigen::Matrix3d::Identity() * (sigma_abias * sigma_abias);

  // 5) Gyro-bias uncertainty 0.01 rad/s
  double sigma_gbias = 0.1;              // rad/s
  P_.block<3,3>(12, 12) = Eigen::Matrix3d::Identity() * (sigma_gbias * sigma_gbias);

  // Magnetometer (yaw) measurement noise – tune as you like
  double sigma_mag_yaw = MathUtils::degreeToRadian(1.0);  // 2° (example)
  V_magyaw_(0,0) = sigma_mag_yaw * sigma_mag_yaw;

  
}

void ErrorStateKalmanFilter::printState() const {
  SPDLOG_INFO("state position : {:.3f}, {:.3f}, {:.3f}", nominal_pos_(0), nominal_pos_(1), nominal_pos_(2));
  SPDLOG_INFO("state velocity : {:.3f}, {:.3f}, {:.3f}", nominal_vel_(0), nominal_vel_(1), nominal_vel_(2));
  SPDLOG_INFO("state quaternion : {:.3f}, {:.3f}, {:.3f}, {:.3f}", nominal_attitude_.w(), nominal_attitude_.x(),
              nominal_attitude_.y(), nominal_attitude_.z());
}

void ErrorStateKalmanFilter::predictWithImu(std::shared_ptr<ImuMeasurement> imuData) {
  std::chrono::duration<double> duration = imuData->timestamp - prev_time_;
  prev_time_ = imuData->timestamp;
  double dt = duration.count();
  if (dt > 0.2) {
    return;
  }
  double dt2 = dt * dt;

  Eigen::Matrix3d R = nominal_attitude_.toRotationMatrix();
  Eigen::Vector3d accel_diff = imuData->acceleration - nominal_accel_bias_;
  Eigen::Vector3d gyro_diff = imuData->angularVelocity - nominal_gyro_bias_;
  

  nominal_pos_ += nominal_vel_ * dt + 0.5 * (R * accel_diff + gravity_vector_) * dt2;
  nominal_vel_ += (R * accel_diff + gravity_vector_) * dt;
  Eigen::Matrix3d dR = MathUtils::rotationVectorToRotationMatrix(gyro_diff * dt);
  nominal_attitude_ = Eigen::Quaterniond(R * dR);
  nominal_attitude_.normalize();

  F_x_.block<3,3>(0,3)  = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3,3>(3,6)  = -R * MathUtils::vec3ToSkewSymmetric(accel_diff) * dt;
  F_x_.block<3,3>(3,9)  = -R * dt;
  F_x_.block<3,3>(6,6)  = dR.transpose();
  F_x_.block<3,3>(6,12) = -Eigen::Matrix3d::Identity()*dt;

  P_ = F_x_ * P_ * F_x_.transpose() + F_i_ * Q_i_ * F_i_.transpose() * dt2;
}



// ─────────────────────────────────────────────────────────────────────────────
// GNSS update:  p (ENU)  +  v (ENU)   with “low-speed” safeguard
// ─────────────────────────────────────────────────────────────────────────────
void ErrorStateKalmanFilter::updateWithGnss(const Eigen::Vector3d& lla_now)
{
  /* 0)  Position & raw (possibly noisy) velocity in ENU -------------------- */
  const auto   t_now   = std::chrono::steady_clock::now();
  Eigen::Vector3d p_enu = llaToEnu(lla_now);

  Eigen::Vector3d v_enu = Eigen::Vector3d::Zero();
  if (have_prev_fix_) {
      double dt = std::chrono::duration<double>(t_now - prev_fix_time_).count();
      if (dt > 1e-3)                     // protect against dt = 0
          v_enu = (p_enu - prev_fix_enu_) / dt;
  }
  /*  -- store for next call ------------------------------------------------- */
  prev_fix_time_ = t_now;
  prev_fix_enu_  = p_enu;
  have_prev_fix_ = true;

  /* 1)  Measurement vector z ------------------------------------------------ */
  Eigen::Matrix<double,6,1> z;
  z << p_enu, v_enu;

  /* 2)  Build Jacobian H (same for all GNSS updates) ------------------------ */
  J_true_error_.block<4,3>(6,6) = computeQuatJacobiToErrorQuat();

  H_x_.setZero();
  H_x_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();   // ∂p/∂p̃
  H_x_.block<3,3>(3,3) = Eigen::Matrix3d::Identity();   // ∂v/∂ṽ
  H_ = H_x_ * J_true_error_;                            // 6×15
  Eigen::Matrix<double,15,6> Ht = H_.transpose();

  /* 3)  Build measurement-noise R  (position + velocity) -------------------- */
  constexpr double SIGMA_P = 10.0;    // [m]   horizontal RMS of the receiver
  constexpr double SIGMA_V_HIGH = 5.0; // [m/s] RMS when speed is “trustable”
  constexpr double SIGMA_V_LOW  = 5.00; // [m/s] inflated RMS when too slow

  const double v_hor = v_enu.head<2>().norm();
  bool   use_vel = (v_hor > 2.0) || (v_hor < 5.0) ;                 // ← speed threshold
  use_vel = false;

  V_.setZero();
  V_.block<3,3>(0,0) = SIGMA_P*SIGMA_P * Eigen::Matrix3d::Identity();
  V_.block<3,3>(3,3) =
          (use_vel ? SIGMA_V_HIGH*SIGMA_V_HIGH
                   : SIGMA_V_LOW *SIGMA_V_LOW) * Eigen::Matrix3d::Identity();

  /*  –– If velocity is NOT trusted, kill its rows in H and innovation ––    */
  if (!use_vel) {
      // std::cout << "velocity is not trusted" << std::endl; 
      H_.block<3,15>(3,0).setZero();     // rows 3..5 gone   
      z.segment<3>(3).setZero();         // innovation will be 0
      Ht = H_.transpose();               // keep H/Hᵀ consistent
  }

  /* 4)  Innovation y -------------------------------------------------------- */
  Eigen::Matrix<double,6,1> y;
  y.block<3,1>(0,0) = p_enu - nominal_pos_;
  y.block<3,1>(3,0) = v_enu - nominal_vel_;
  if (!use_vel)      y.segment<3>(3).setZero();     // velocity ignored when too slow

  /* 5)  Standard EKF gain & update ----------------------------------------- */
  Eigen::Matrix<double,6,6> S = H_ * P_ * Ht + V_;
  K_ = P_ * Ht * S.ldlt().solve(Eigen::Matrix<double,6,6>::Identity());

  error_x_ = K_ * y;
  P_       = (Mat15d::Identity() - K_ * H_) * P_;

  injectErrorToNominal();
  resetErrorState();
}



void ErrorStateKalmanFilter::updateWithWheelEncoder(const double& encoderData) {
  // V_.block<3, 3>(0, 0) = gnssData->positionCovariance;
  V_encoder_(0,0) = 30*30;
  J_true_error_.block<4, 3>(6, 6) = computeQuatJacobiToErrorQuat();
  H_x_encoder_.block<1, 3>(0,3) = computeWheelEncoderJacobian();
  H_encoder_ = H_x_encoder_ * J_true_error_;

  Eigen::Matrix<double, 1, 1> S_encoder = H_encoder_ * P_ * H_encoder_.transpose() + V_encoder_;
  K_encoder_ = P_ * H_encoder_.transpose() * S_encoder.ldlt().solve(Eigen::Matrix<double, 1, 1>::Identity()); // compute inverse via Cholesky decomposition

  

  error_x_ = K_encoder_ * (encoderData - nominal_vel_.norm());
  P_ = (Mat15d::Identity() - K_encoder_ * H_encoder_) * P_;

  injectErrorToNominal();
  resetErrorState();
}

void ErrorStateKalmanFilter::updateWithMagYaw(const double& mag_yaw_rad) {

  if (is_mag_yaw_initialized_) {
    Eigen::Vector3d rpy = nominal_attitude_
        .toRotationMatrix()
        .eulerAngles(0, 1, 2);
    double roll  = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2];
    
    

    return; // currently we are not using magnetometer during drive
      // ── 1. Build H for this scalar measurement ────────────────────────────────
    J_true_error_.block<4,3>(6,6) = computeQuatJacobiToErrorQuat();  // refresh
    H_x_magyaw_.setZero();
    H_x_magyaw_.block<1,3>(0,6) = computeMagYawJacobian();
    H_magyaw_ = H_x_magyaw_ * J_true_error_;                         // (1×15)

    // ── 2. Kalman gain for the 1-D measurement ───────────────────────────────
    Eigen::Matrix<double,1,1> S = H_magyaw_ * P_ * H_magyaw_.transpose() + V_magyaw_;
    K_magyaw_ = (P_ * H_magyaw_.transpose()) * S.ldlt().solve(Eigen::Matrix<double,1,1>::Identity());

    // ── 3. Innovation (wrapped to ±π) ────────────────────────────────────────
    Eigen::Quaterniond q = nominal_attitude_.normalized();
    double yaw_nominal = std::atan2( 2.0*(q.w()*q.z() + q.x()*q.y()),
                                 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()) );
    double innov = mag_yaw_rad - yaw_nominal;
    // wrap to (-π, π]
    innov = std::atan2(std::sin(innov), std::cos(innov));

    // ── 4. Error-state update ────────────────────────────────────────────────
    error_x_ = K_magyaw_ * innov;
    P_ = (Mat15d::Identity() - K_magyaw_ * H_magyaw_) * P_;

    // ── 5. Inject & reset ────────────────────────────────────────────────────
    injectErrorToNominal();
    resetErrorState();
  }
  else {
    
    // 2) Extract current roll & pitch from the nominal attitude quaternion
    //    (returns [roll, pitch, yaw])
    Eigen::Vector3d rpy = nominal_attitude_
        .toRotationMatrix()
        .eulerAngles(0, 1, 2);
    double roll  = rpy[0];
    double pitch = rpy[1];

    // 3) Rebuild the quaternion with the measured yaw but keep roll & pitch
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(mag_yaw_rad, Eigen::Vector3d::UnitZ());
    nominal_attitude_ = q.normalized();

    // 4) Inflate yaw covariance so subsequent updates can fine‐tune easily
    //    Here we set yaw variance to (30°)^2 ≈ 0.27 rad²
    P_.block<3,3>(6,6) = 0.27 * Eigen::Matrix3d::Identity();

    // 5) Mark initialization done
    is_mag_yaw_initialized_ = true;


    std::cout << "[EKF] Started (mag yaw aligned)\n" << mag_yaw_rad*180.0/ M_PI << std::endl;
  }
}


Eigen::Matrix<double, 1, 3> ErrorStateKalmanFilter::computeWheelEncoderJacobian() {
  Eigen::Matrix<double, 1, 3> wheelEncoderJacobian = Eigen::Matrix<double, 1, 3>::Zero();
  wheelEncoderJacobian(0, 0) = nominal_vel_(0) / nominal_vel_.norm();
  wheelEncoderJacobian(0, 1) = nominal_vel_(1) / nominal_vel_.norm();
  wheelEncoderJacobian(0, 2) = nominal_vel_(2) / nominal_vel_.norm();
  return wheelEncoderJacobian;
}

Eigen::Matrix<double, 1, 3> ErrorStateKalmanFilter::computeMagYawJacobian() const {
  // For small attitude errors, δψ ≈ δθ_z (rotation about navigation-frame Z).
  Eigen::Matrix<double, 1, 3> J;
  J << 0.0, 0.0, 1.0;
  return J;
}


Eigen::Matrix<double, 4, 3> ErrorStateKalmanFilter::computeQuatJacobiToErrorQuat() {
  Eigen::Matrix<double, 4, 3> quat_true_error = Eigen::Matrix<double, 4, 3>::Zero();
  quat_true_error.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity() * 0.5;
  quat_true_error = MathUtils::quatToLeftProductMatrix(nominal_attitude_) * quat_true_error;
  return quat_true_error;
}

Eigen::Matrix<double, 6, 1> ErrorStateKalmanFilter::computeHx() {
  Eigen::Matrix<double, 6, 1> hx;
  hx.block<3, 1>(0, 0) = nominal_pos_;
  hx.block<3, 1>(3, 0) = nominal_vel_;
  return hx;
}

Eigen::Vector3d ErrorStateKalmanFilter::llaToEnu(const Eigen::Vector3d& llaPosition) {
  if (!local_cartesian_initialized_) {
    local_cartesian_.Reset(llaPosition(0), llaPosition(1), llaPosition(2));
    local_cartesian_initialized_ = true;
  }
  Eigen::Vector3d enuPosition;
  local_cartesian_.Forward(llaPosition(0), llaPosition(1), llaPosition(2), enuPosition(0), enuPosition(1),
                           enuPosition(2));
  return enuPosition;
}

Eigen::Vector3d ErrorStateKalmanFilter::enuToLla(const Eigen::Vector3d& enuPosition) {
    if (!local_cartesian_initialized_) {
        SPDLOG_INFO("Local Cartesian coordinate system is not initialized. Call llaToEnu first.");
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d llaPosition;
    local_cartesian_.Reverse(enuPosition(0), enuPosition(1), enuPosition(2), 
                             llaPosition(0), llaPosition(1), llaPosition(2));
    return llaPosition;
}

void ErrorStateKalmanFilter::injectErrorToNominal() {
  nominal_pos_ += error_x_.block<3, 1>(0, 0);
  nominal_vel_ += error_x_.block<3, 1>(3, 0);
  nominal_attitude_ = nominal_attitude_ * MathUtils::rotationVectorToQuaternion(error_x_.block<3, 1>(6, 0));
  nominal_attitude_.normalize();
  nominal_accel_bias_ += error_x_.block<3, 1>(9, 0);
  nominal_gyro_bias_ += error_x_.block<3, 1>(12, 0);
}

void ErrorStateKalmanFilter::resetErrorState() {
  G_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * MathUtils::vec3ToSkewSymmetric(error_x_.block<3, 1>(6, 0));
  P_ = G_ * P_ * G_.transpose();
  error_x_.setZero();
}
