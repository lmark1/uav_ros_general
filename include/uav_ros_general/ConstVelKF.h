#ifndef CONST_VEL_KF_H
#define CONST_VEL_KF_H

#include "kf_base.h"
#include <Eigen/Dense>
#include <uav_ros_general/VelocityEstimationParametersConfig.h>
#include <iostream>

class ConstVelKF
    : public kf_base<uav_ros_general::VelocityEstimationParametersConfig, 1, 2> {
public:
  /**
   * @brief A constructor.
   * Initializes all private variables.
   */
  ConstVelKF() : kf_base() { initializeInternal(); }
  ConstVelKF(std::string name, ros::NodeHandle& nh) : kf_base(name) { 
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
    Eigen::Matrix2f A;
    A << 1, dt, 0, 1;
    x_k_ = A * x_k_;

    Eigen::Matrix2f Q;
    Q << pow(process_noise_position_stddev_, 2), 0, 0,
        pow(process_noise_velocity_stddev_, 2);
    P_k_ = A * P_k_ * A.transpose() + Q;
  }

  /**
   * @brief Corrects the state using the measurement vector
   * 
   * @param measurement_vector 
   */
  void measurementUpdate(meas_t measurement_vector) override {
    auto S = H_ * P_k_ * H_.transpose() + R_;
    auto K = P_k_ * H_.transpose() / S;
    auto residual = measurement_vector - H_ * x_k_;
    x_k_ = x_k_ + K * residual;
    P_k_ = (IDENTITY - K * H_) * P_k_;
  }

  friend std::ostream &operator<<(std::ostream &out, const ConstVelKF &filt) {
    out << "ConstVelKF parameters are:"
        << "\nMeasure noise=" << filt.R_
        << "\nPosition noise=" << filt.process_noise_position_stddev_
        << "\nVelocity noise=" << filt.process_noise_velocity_stddev_
        << std::endl;
    return out;
  }

  void parametersCallback(typename kf_base::config_t &configMsg,
                          uint32_t /* unnused */) override {
    setMeasureNoise(configMsg.noise_mv);
    setPositionNoise(configMsg.noise_pos);
    setVelocityNoise(configMsg.noise_vel);
    ROS_INFO_STREAM(*this);
  }

  kf_base::config_t initializeParameters(ros::NodeHandle& nh) override {
    ROS_WARN("ConstVelKF::initializeParameters()");
    // Setup dynamic reconfigure server
    float kalmanNoiseMv;
    float kalmanNoisePos;
    float kalmanNoiseVel;
    bool initialized = 
        nh.getParam(getName() + "/kalman/noise_mv", kalmanNoiseMv) &&
        nh.getParam(getName() + "/kalman/noise_pos", kalmanNoisePos) &&
        nh.getParam(getName() + "/kalman/noise_vel", kalmanNoiseVel);

    setMeasureNoise(kalmanNoiseMv);
    setPositionNoise(kalmanNoisePos);
    setVelocityNoise(kalmanNoiseVel);
    ROS_INFO_STREAM(*this);

    if (!initialized)
    {
        ROS_FATAL("ConstVelKF::initializeParameters() - parameter initialization failed.");
        throw std::invalid_argument("ConstVelKF_alt parameters not properly set.");
    }

    kf_base::config_t cfg;
    cfg.noise_mv = kalmanNoiseMv;
    cfg.noise_pos = process_noise_position_stddev_;
    cfg.noise_vel = process_noise_velocity_stddev_;
    return cfg;
  }
  
  void initializeState(meas_t initialState) override {
    x_k_[0] = initialState[0];
    x_k_[1] = 0;
  }

private:

  void initializeInternal() {
    static constexpr auto initial_pos_stddev = 1;
    static constexpr auto initial_vel_stddev = 10;
    static constexpr auto measurement_position_stddev = 1;
    P_k_ << initial_pos_stddev * initial_pos_stddev, 0, 0,
        initial_vel_stddev * initial_vel_stddev;
    R_ = measurement_position_stddev * measurement_position_stddev;
    x_k_ << 0, 0;
    H_ << 1, 0;
    process_noise_position_stddev_ = initial_pos_stddev;
    process_noise_velocity_stddev_ = initial_vel_stddev;
  }

  /**
   * @brief Sets the kalman filter parameter measurement noise
   *
   * @param r Measurement noise
   */
  void setMeasureNoise(float r) { R_ = r * r; }

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

  // Measurement noise covariance
  float R_;
  float process_noise_position_stddev_;
  float process_noise_velocity_stddev_;

  // Measurement matrix
  Eigen::RowVector2f H_;

  // Dynamic model matrix
  Eigen::Matrix2f A_;

  // Identity matrix
  const Eigen::Matrix2f IDENTITY = Eigen::Matrix2f::Identity(2, 2);
};

#endif // CONST_VEL_KF_H
