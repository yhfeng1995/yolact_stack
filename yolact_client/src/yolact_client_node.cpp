//
// Created by yonghui on 19-7-16.
//
#include <ros/ros.h>
#include <ros/console.h>
#include "yolact_client/tum_publisher.h"
#include "yolact_client/yolact_client.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolact_clent_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;
    // client
    MY_SLAM::YolactClient yc(nh);
    ros::spin();
}