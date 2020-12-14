#ifndef KF_BASE_H
#define KF_BASE_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include <dynamic_reconfigure/server.h>

template <typename kalman_config, unsigned int measurement_count, unsigned int state_count>
class kf_base {
public:
    using config_t = kalman_config;
    using meas_t = Eigen::Matrix<float, measurement_count, 1>;
    using state_t = Eigen::Matrix<float, state_count, 1>;
    using cov_t = Eigen::Matrix<float, state_count, state_count>;

    static constexpr auto measure_size = measurement_count;
    static constexpr auto state_size = state_count;
    static constexpr auto max_invalid_time = 2;

    explicit kf_base(std::string name) : 
        name_(name), 
        _kalmanInitialized(false), 
        _timeInvalid(0),
        _configServer {_configMutex, ros::NodeHandle("kalman_ns/" + name + "_config")} {}
    kf_base() : kf_base("default") {}

    virtual void parametersCallback(config_t& configMsg, unsigned int /* unnused */) = 0;
    virtual config_t initializeParameters(ros::NodeHandle& nh) = 0;
    virtual void modelUpdate(double dt) = 0;
    virtual void measurementUpdate(meas_t measurement_vector) = 0;
    virtual void initializeState(meas_t initialState) = 0;

    const state_t& getState() const { return x_k_; }
    const cov_t& getStateCovariance() const { return P_k_; }

    void estimateState(double dt, std::array<float, measure_size> measurements, bool newMeasurementFlag)
    {   
        // Reset filtered distance if filter is not initialized
        if (!_kalmanInitialized){
            this->resetState();
        }

        // Check if initialization failed
        if (!_kalmanInitialized && !newMeasurementFlag){
            ROS_WARN_THROTTLE(5.0, "KalmanWrapper - Failed to initialize");
            return;
        }

        // Check if initialization should take place
        if (!_kalmanInitialized && newMeasurementFlag) {
            _kalmanInitialized = true;
            this->initializeState(meas_t(measurements.data()));
            ROS_INFO_STREAM("KalmanWrapper " << name_ << " - Initialized.");
        }

        // Do model update
        this->modelUpdate(dt);

        // Do measure update if everything is valid
        if (newMeasurementFlag)
        {
            // ROS_INFO("KalmanFilter - New measurement! update called");
            this->measurementUpdate(meas_t(measurements.data()));
            _timeInvalid = 0;
        }
        else
        {
            // Increase time invalid
            // ROS_WARN("KalmanFilter - doing only model update");
            _timeInvalid += dt;
        }

        // Check if invalid time reached maximum
        if (_timeInvalid > max_invalid_time)
        {
            this->resetState();
            ROS_FATAL_STREAM("KalmanWrapper " << name_ << " - Max invalid time reached.");
            return;
        }
    }

    void setupReconfigureServer(ros::NodeHandle& nh, const config_t& cfg) {
      _configServer.updateConfig(cfg);
      _configParamCb = boost::bind(
          &kf_base::parametersCallback, this, _1, _2);
      _configServer.setCallback(_configParamCb);
    }

    void resetState()
    {
        _kalmanInitialized = false;
        _timeInvalid = 0;
    }

    const std::string& getName() const {
        return name_;
    }
private:
    std::string name_;

    /** Define Dynamic Reconfigure parameters **/
    boost::recursive_mutex _configMutex;
    dynamic_reconfigure::Server<config_t> _configServer;
    typename dynamic_reconfigure::Server<config_t>::CallbackType _configParamCb;

    /** Flag signaling that kalman filter is initialized. */
    bool _kalmanInitialized;

    /** Time passed while measurements are invalid. */
    double _timeInvalid;
  
protected:
    state_t x_k_;
    cov_t P_k_;
    
};

#endif // KF_BASE_H