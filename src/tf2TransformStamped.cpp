//
// Created by marko on 12.02.19..
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "map2transform");

    ros::NodeHandle node;

    ros::Publisher map_position = node.advertise<geometry_msgs::TransformStamped>("eaglered/position", 1);

    tf::TransformListener listener;

    ros::Rate rate(100.0);
    while (node.ok()){
        tf::StampedTransform transform;
        geometry_msgs::TransformStamped position;
        try{
            listener.lookupTransform("/map","/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        position.header.stamp = transform.stamp_;
        position.header.frame_id = transform.frame_id_;
        position.child_frame_id = transform.child_frame_id_;
        position.transform.translation.x = transform.getOrigin().x();
        position.transform.translation.y = transform.getOrigin().y();
        position.transform.translation.z = transform.getOrigin().z();
        position.transform.rotation.x = transform.getRotation().x();
        position.transform.rotation.y = transform.getRotation().y();
        position.transform.rotation.z = transform.getRotation().z();
        position.transform.rotation.w = transform.getRotation().w();

        map_position.publish(position);

        rate.sleep();
    }

    return 0;
};