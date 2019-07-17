//
// Created by yonghui on 19-7-17.
//

#include <ros/ros.h>
#include "yolact_client/tum_publisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tum_publish_node");
    ros::NodeHandle nh;
    MY_SLAM::TumPublisher tp(nh);
    bool bPublishSccuess = true;
    ros::Rate r(10);
    while (ros::ok() && bPublishSccuess)
    {
        ROS_INFO("publish once...");
        bPublishSccuess = tp.PublishNext();
        ros::spinOnce();
        r.sleep();
    }
}