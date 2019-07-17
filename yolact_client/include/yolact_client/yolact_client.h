//
// Created by yonghui on 19-7-16.
//

#ifndef YOLACT_CLIENT_YOLACT_CLIENT_H
#define YOLACT_CLIENT_YOLACT_CLIENT_H

// cv2
#include <opencv2/core.hpp>
// ros
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
// stl
#include <string>
#include <vector>
// mine
#include "yolact_client/drawer.h"


namespace MY_SLAM
{
    class YolactClient
    {
    public:
        YolactClient(ros::NodeHandle &nh_);
    
    protected:
        /**
         * @brief Yolact callback function
         *
         * @param msg
         */
        void YolactMaskCb(const sensor_msgs::CompressedImageConstPtr &msg);
        
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher pub_visual_;
        ros::Subscriber sub_;
        ros::ServiceClient client_;
        ros::Publisher pub_result_;  ///<@brief Yolact result publisdher
        
        // visual
        bool mbVisual;
        Drawer mUtils;
    };
}



#endif //YOLACT_CLIENT_YOLACT_CLIENT_H
