#ifndef CONST_ACC_KF_H
#define CONST_ACC_KF_H

#include "kf_base.h"
#include <iostream>
#include <uav_ros_general/KalmanConstAccParametersConfig.h>

class ConstAccKF : 
  public kf_base<uav_ros_general::KalmanConstAccParametersConfig, 2, 3> {
public:
  /**
   * @brief A constructor.
   * Initializes all private variables.
   */
  ConstAccKF() :  kf_base() { initializeInternal();  }
  ConstAccKF(std::string name, ros::NodeHandle& nh) : kf_base(name) {
    initializeInternal();
    auto cfg = initializeParameters(nh);
    setupReconfigureServer(nh, cfg);
  }

  /**
   * @brief Method for prediction. Updates the states using simplified model
   * (integratior)
   *
   * @param dt Time from last update
   */
  void modelUpdate(double dt) override {
    Eigen::Matrix3f A;
    A << 1, dt, dt * dt / 2., 
      0, 1., dt, 
      0, 0, 1.;
    x_k_ = A * x_k_;

    Eigen::Matrix3f Q;
    Q << pow(process_noise_position_stddev_, 2), 0, 0,
      0, pow(process_noise_velocity_stddev_, 2), 0,
      0, 0, pow(process_noise_acceleration_stddev_, 2);
    P_k_ = A * P_k_ * A.transpose() + Q;
  }

  /**
   * @brief Corrects the states using new measurement.
   *
   * @param pos_m New measurement
   */
  void measurementUpdate(meas_t measurement_vector) override {
    auto S = H_ * P_k_ * H_.transpose() + R_;         // 2x2
    auto K = ( P_k_ * H_.transpose() ) * S.inverse(); // 3x2
    auto residual = measurement_vector - H_ * x_k_;   // 2x1
    x_k_ = x_k_ + K * residual;                       // 3x1
    P_k_ = (IDENTITY - K * H_) * P_k_;                // 3x3
  }

  /**
   * @brief Method sets initial position value
   *
   * @param pos Position value
   */
  void initializeState(meas_t initialState) override { 
    x_k_[0] = initialState[0];
    x_k_[1] = 0;
    x_k_[2] = initialState[1];
  }

  friend std::ostream &operator<<(std::ostream &out, const ConstAccKF &filt) {
    out << "Kalman Filter parameters are:"
        << "\nMeasure noise=" << filt.R_
        << "\nPosition noise=" << filt.process_noise_position_stddev_
        << "\nVelocity noise=" << filt.process_noise_velocity_stddev_
        << "\nAcceleration noise=" << filt.process_noise_acceleration_stddev_
        << std::endl;
    return out;
  }

void parametersCallback(config_t& configMsg, unsigned int /* unnused */) override {
  ROS_INFO("ConstAccKF::parametersCallback");
  ROS_INFO_STREAM(*this);
  setMeasurePositionNoise(configMsg.noise_pos_mv);
  setMeasureAccelerationNoise(configMsg.noise_acc_mv);
  setPositionNoise(configMsg.noise_pos_proc);
  setVelocityNoise(configMsg.noise_vel_proc);
  setAccelerationNoise(configMsg.noise_acc_proc);
  ROS_INFO_STREAM(*this);
}

config_t initializeParameters(ros::NodeHandle& nh) override {
  ROS_WARN("ConstAccKF::initializeParameters()");
  // Setup dynamic reconfigure server

  float kalmanNoisePosMv;
  float kalmanNoiseAccMv;
  float kalmanNoisePosProc;
  float kalmanNoiseVelProc;
  float kalmanNoiseAccProc;

  bool initialized = 
      nh.getParam(getName() + "/kalman/noise_pos_mv", kalmanNoisePosMv) &&
      nh.getParam(getName() + "/kalman/noise_acc_mv", kalmanNoiseAccMv) &&
      nh.getParam(getName() + "/kalman/noise_pos_proc", kalmanNoisePosProc) &&
      nh.getParam(getName() + "/kalman/noise_vel_proc", kalmanNoiseVelProc) &&
      nh.getParam(getName() + "/kalman/noise_acc_proc", kalmanNoiseAccProc);

  setMeasurePositionNoise(kalmanNoisePosMv);
  setMeasureAccelerationNoise(kalmanNoiseAccProc);
  setPositionNoise(kalmanNoisePosProc);
  setVelocityNoise(kalmanNoiseVelProc);
  setAccelerationNoise(kalmanNoiseAccProc);
  ROS_INFO_STREAM(*this);

  if (!initialized)
  {
      ROS_FATAL("KalmanWrapper::initializeParameters() - parameter initialization failed.");
      throw std::invalid_argument("KalmanWrapper parameters not properly set.");
  }

  kf_base::config_t cfg;
  cfg.noise_pos_mv = kalmanNoisePosMv;
  cfg.noise_acc_mv = kalmanNoiseAccMv;
  cfg.noise_pos_proc = kalmanNoisePosProc;
  cfg.noise_vel_proc = kalmanNoiseVelProc;
  cfg.noise_acc_proc = kalmanNoiseAccProc;
  return cfg;
}

private:
  void initializeInternal() {
    static constexpr auto initial_pos_stddev = 1;
    static constexpr auto initial_vel_stddev = 10;
    static constexpr auto initial_acc_stddev = 10;
    static constexpr auto measurement_position_stddev = 1;
    static constexpr auto measurement_acc_stddev = 10;

    P_k_ << initial_pos_stddev * initial_pos_stddev, 0, 0,
        0, initial_vel_stddev * initial_vel_stddev, 0,
        0, 0, initial_acc_stddev * initial_acc_stddev;
    R_ << measurement_position_stddev * measurement_position_stddev, 0, 
      0, measurement_acc_stddev * measurement_acc_stddev;
    x_k_ << 0, 0;
    H_ << 1, 0, 0, 0, 0, 1;

    process_noise_position_stddev_ = initial_pos_stddev;
    process_noise_velocity_stddev_ = initial_vel_stddev;
    process_noise_acceleration_stddev_ = initial_acc_stddev;
  }

  /**
   * @brief Sets the kalman filter parameter measurement noise
   *
   * @param r Measurement position noise
   */
  void setMeasurePositionNoise(float r) { R_(0,0) = r * r; }

  /**
   * @brief Set the Measure Acceleration Noise object.
   * 
   * @param r Measurement acceleration noise
   */
  void setMeasureAccelerationNoise(float r) { R_(1,1) = r * r; }
  
  /**
   * @brief Method sets the kalman filter process noise (position)
   *
   * @param q Process noise (position)
   */
  void setPositionNoise(float q) { process_noise_position_stddev_ = q; }

  /**
   * @brief Method sets the kalman filter process noise (velocity)
   *
   * @param q Process noise (velocity)
   */
  void setVelocityNoise(float q) { process_noise_velocity_stddev_ = q; }

  /**
   * @brief Set the Acceleration Noise object.
   * 
   * @param q Process noise (acceleration)
   */
  void setAccelerationNoise(float q) { process_noise_acceleration_stddev_ = q; }

  // Measurement noise covariance
  Eigen::Matrix2f R_;

  float process_noise_position_stddev_;
  float process_noise_velocity_stddev_;
  float process_noise_acceleration_stddev_;

  // Measurement matrix
  Eigen::Matrix<float, 2, 3> H_;

  // Dynamic model matrix
  Eigen::Matrix3f A_;

  // Identity matrix
  const Eigen::Matrix3f IDENTITY = Eigen::Matrix3f::Identity(3, 3);
};

#endif // CONST_ACC_KF_H
