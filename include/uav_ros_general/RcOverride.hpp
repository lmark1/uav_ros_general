/*
 * RcOverride.h
 *
 *  Created on: Feb 8, 2019
 *      Author: lmark
 */

#ifndef RCOVERRIDE_H_
#define RCOVERRIDE_H_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "mavros_msgs/OverrideRCIn.h"
#include <mavros_msgs/RCIn.h>
#include <vector>

class RcOverride
{

public:
  RcOverride();
  virtual ~RcOverride();

  bool overrideONCallback(std_srvs::Empty::Request &request,
    std_srvs::Empty::Response &response);
  bool overrideOFFCallback(std_srvs::Empty::Request &request,
    std_srvs::Empty::Response &response);
  void setOverridePublisher(ros::Publisher pub);
  void setOverrideIndex(int index);
  void setOverrideValue(int value);
  void rcCallback(const mavros_msgs::RCIn &msg);

private:
  ros::Publisher override_pub;
  int override_index_ = -1;
  int override_value_ = -1;
  std::vector<int> override_indices_;

  void publishOverrideMessage();
};

#endif /* RCOVERRIDE_H_ */
