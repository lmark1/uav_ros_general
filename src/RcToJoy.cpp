/*
 * RcToJoy.cpp
 *
 *  Created on: Jun 23, 2017
 *      Author: thaus
 */

#include <uav_ros_general/RcToJoy.hpp>
#include <uav_ros_lib/nonlinear_filters.hpp>

RcToJoy::RcToJoy()
  : rc_channel_throttle_(rc_channels::RC_CHANNEL_THROTTLE),
    rc_channel_roll_(rc_channels::RC_CHANNEL_ROLL),
    rc_channel_pitch_(rc_channels::RC_CHANNEL_PITCH),
    rc_channel_yaw_(rc_channels::RC_CHANNEL_YAW),
    rc_channel_rc_on_(rc_channels::RC_CHANNEL_RC_ON),
    rc_channel_mode_(rc_channels::RC_CHANNEL_MODE),
    rc_inspection_mode_(rc_channels::RC_INSPECTION_MODE),
    rc_sequence_mode_(rc_channels::RC_SEQUENCE_MODE),

    joy_axis_throttle_(joy_axes::JOY_AXIS_THROTTLE),
    joy_axis_roll_(joy_axes::JOY_AXIS_ROLL), joy_axis_pitch_(joy_axes::JOY_AXIS_PITCH),
    joy_axis_yaw_(joy_axes::JOY_AXIS_YAW), joy_axis_mode_(joy_axes::JOY_AXIS_MODE),
    joy_axis_sequence_(joy_axes::JOY_SEQUENCE_MODE),
    joy_button_rc_on_(joy_buttons::JOY_BUTTON_RC_ON),
    joy_inspection_mode_(joy_buttons::JOY_INSPECTION_MODE),

    _xDeadzone(0), _yDeadzone(0), _zDeadzone(0), _yawDeadzone(0)
{
  // TODO Auto-generated constructor stub
}

RcToJoy::~RcToJoy()
{
  // TODO Auto-generated destructor stub
}

void RcToJoy::setJoyPublisher(ros::Publisher joy_pub) { joy_pub_ = joy_pub; }


void RcToJoy::rcCallback(const mavros_msgs::RCIn &msg)
{

  // first check is rc msg contains any channel value
  if (msg.channels.size() > 0) {

    sensor_msgs::Joy joy_msg;
    std::vector<float> axis_array(12, 0);// length 12, init to 0
    std::vector<int> button_array(12, 0);// length 12, init to 0

    joy_msg.axes = axis_array;
    joy_msg.buttons = button_array;

    joy_msg.header = msg.header;
    joy_msg.axes[joy_axis_throttle_] =
      nonlinear_filters::deadzone(
        rcChannelToJoyAxis(msg.channels[rc_channel_throttle_]), -_zDeadzone, _zDeadzone)
      / (joy_axes::JOY_AXIS_VALUE_MAX - _zDeadzone);
    joy_msg.axes[joy_axis_roll_] =
      nonlinear_filters::deadzone(
        -rcChannelToJoyAxis(msg.channels[rc_channel_roll_]), -_yDeadzone, _yDeadzone)
      / (joy_axes::JOY_AXIS_VALUE_MAX - _yDeadzone);
    joy_msg.axes[joy_axis_pitch_] =
      nonlinear_filters::deadzone(
        -rcChannelToJoyAxis(msg.channels[rc_channel_pitch_]), -_xDeadzone, _xDeadzone)
      / (joy_axes::JOY_AXIS_VALUE_MAX - _xDeadzone);
    joy_msg.axes[joy_axis_yaw_] =
      -nonlinear_filters::deadzone(
        rcChannelToJoyAxis(msg.channels[rc_channel_yaw_]), -_yawDeadzone, _yawDeadzone)
      / (joy_axes::JOY_AXIS_VALUE_MAX - _yawDeadzone);
    joy_msg.axes[joy_axis_mode_] = rcChannelToJoyAxis(msg.channels[rc_channel_mode_]);
    joy_msg.axes[joy_axis_sequence_] =
      rcChannelToJoyAxis(msg.channels[rc_sequence_mode_]);
    joy_msg.buttons[joy_button_rc_on_] =
      rcChannelToJoyButton(msg.channels[rc_channel_rc_on_]);
    joy_msg.buttons[joy_inspection_mode_] =
      rcChannelToJoyButton(msg.channels[rc_inspection_mode_]);

    joy_msg.header.stamp = ros::Time::now();
    joy_pub_.publish(joy_msg);
  }
}

float RcToJoy::rcChannelToJoyAxis(float rc_channel_value)
{
  float joy_value;
  joy_value =
    (rc_channel_value
      - (rc_channels::RC_CHANNEL_VALUE_MAX + rc_channels::RC_CHANNEL_VALUE_MIN) / 2.0)
    / ((rc_channels::RC_CHANNEL_VALUE_MAX - rc_channels::RC_CHANNEL_VALUE_MIN) / 2.0);
  if (joy_value < joy_axes::JOY_AXIS_VALUE_MIN)
    joy_value = joy_axes::JOY_AXIS_VALUE_MIN;
  else if (joy_value > joy_axes::JOY_AXIS_VALUE_MAX)
    joy_value = joy_axes::JOY_AXIS_VALUE_MAX;

  return joy_value;
}


uint8_t RcToJoy::rcChannelToJoyButton(float rc_channel_value)
{

  uint8_t joy_value;

  if (rc_channel_value
      > (rc_channels::RC_CHANNEL_VALUE_MAX + rc_channels::RC_CHANNEL_VALUE_MIN) / 2.0
          + 100)
    joy_value = joy_buttons::JOY_BUTTON_VALUE_ON;
  else
    joy_value = joy_buttons::JOY_BUTTON_VALUE_OFF;

  return joy_value;
}

void RcToJoy::initializeParameters(ros::NodeHandle &nh)
{
  bool initialized = nh.getParam("/joy/deadzone/x", _xDeadzone)
                     && nh.getParam("/joy/deadzone/y", _yDeadzone)
                     && nh.getParam("/joy/deadzone/z", _zDeadzone)
                     && nh.getParam("/joy/deadzone/yaw", _yawDeadzone);
  ROS_INFO("Deadzone values:\nx: %.2f \ny: %.2f \nz: %.2f \nyaw: %.2f",
    _xDeadzone,
    _yDeadzone,
    _zDeadzone,
    _yawDeadzone);
  if (!initialized) {
    ROS_FATAL("RcToJoy::initializeParameters - deadzone parameters not loaded.");
    throw std::runtime_error("Deadzone parameters are not initialized.");
  }
}
