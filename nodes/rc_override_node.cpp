#include <uav_ros_general/RcOverride.hpp>

/**
 *	Main override node.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rc_override_node");
  RcOverride rcOverride;

  ros::NodeHandle nh;
  ros::Subscriber subRcIn =
    nh.subscribe("mavros/rc/in", 1, &RcOverride::rcCallback, &rcOverride);
  ros::Publisher overridePub =
    nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1);
  ros::ServiceServer service_on = nh.advertiseService(
    "magnet/override_ON", &RcOverride::overrideONCallback, &rcOverride);
  ros::ServiceServer service_off = nh.advertiseService(
    "magnet/override_OFF", &RcOverride::overrideOFFCallback, &rcOverride);

  ros::NodeHandle nh_("~");
  int overrideIndex = -1;
  int overrideValue = -1;
  nh_.getParam("override_index", overrideIndex);
  nh_.getParam("override_value", overrideValue);

  std::cout << "Override index: " << overrideIndex << std::endl;
  std::cout << "Override value: " << overrideValue << std::endl;

  rcOverride.setOverridePublisher(overridePub);
  rcOverride.setOverrideIndex(overrideIndex);
  rcOverride.setOverrideValue(overrideValue);

  ros::spin();
  return 0;
}
