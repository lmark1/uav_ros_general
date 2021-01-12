#include <uav_ros_general/RcOverride.hpp>
#include <mavros_msgs/OverrideRCIn.h>

RcOverride::RcOverride() { ROS_INFO("Hello from override constructor"); }

RcOverride::~RcOverride() {}

bool RcOverride::overrideOFFCallback(std_srvs::Empty::Request &request,
  std_srvs::Empty::Response &response)
{
  ROS_INFO("Override OFF");
  override_indices_ = { 0, 0, 0, 0, 0, 0, 0, 0 };
  return true;
}

bool RcOverride::overrideONCallback(std_srvs::Empty::Request &request,
  std_srvs::Empty::Response &response)
{
  ROS_INFO("Override ON");
  override_indices_ = { 0, 0, 0, 0, 0, 0, 0, 0 };
  override_indices_[override_index_] = override_value_;
  return true;
}

void RcOverride::rcCallback(const mavros_msgs::RCIn &msg) { publishOverrideMessage(); }

void RcOverride::publishOverrideMessage()
{
  mavros_msgs::OverrideRCIn newMsg;
  std::copy(override_indices_.begin(), override_indices_.end(), newMsg.channels.elems);
  override_pub.publish(newMsg);
}

void RcOverride::setOverridePublisher(ros::Publisher pub) { override_pub = pub; }

void RcOverride::setOverrideIndex(int index) { override_index_ = index; }

void RcOverride::setOverrideValue(int value) { override_value_ = value; }
