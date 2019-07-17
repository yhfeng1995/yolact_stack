//
// Created by yonghui on 19-7-16.
//

#include "yolact_client/tum_publisher.h"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

namespace MY_SLAM
{
    TumPublisher::TumPublisher(ros::NodeHandle &nh) :
    nh_(nh), it_(nh), mnCurrentIdx(0)
    {
        private_nh_ = ros::NodeHandle("~");
        pub_ = it_.advertise("image", 1);
        private_nh_.getParam("dataset_dir", mstrDataSetDir);
        private_nh_.getParam("seq_name", mstrSeqName);
        private_nh_.getParam("associate_path", mstrAssociatedPath);
        LoadImagesRGBTUM();
    }
    
    
    bool TumPublisher::PublishNext()
    {
        // empty list
        if (mvstrImageFilenames.empty())
        {
            ROS_ERROR("TUM Publisher image list is empty!");
            return false;
        }
        
        // complete publish
        if (mvstrImageFilenames.size() == mnCurrentIdx)
        {
            ROS_INFO("TUM Publisher complete!");
            return false;
        }

        // read image
        string strImagePath = mstrDataSetDir + "/" + mstrSeqName + "/" + mvstrImageFilenames[mnCurrentIdx];
        cv::Mat imRGB = cv::imread(strImagePath, -1);
        if (imRGB.empty())
        {
            ROS_ERROR("Fail to load image at %s", strImagePath.c_str());
            return false;
        }
        
        // convert to message and publish
        std_msgs::Header msgHeader;
        msgHeader.stamp = ros::Time(mvTimestamps[mnCurrentIdx]);
        msgHeader.frame_id = "image";
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(msgHeader, sensor_msgs::image_encodings::BGR8, imRGB));
        pub_.publish(cv_ptr->toImageMsg());
        mnCurrentIdx++;
        return true;
    }
    
    
    void TumPublisher::LoadImagesRGBTUM()
    {
        std::ifstream fAssociation;
//        ROS_ERROR("associate path: %s", mstrAssociatedPath.c_str());
        fAssociation.open(mstrAssociatedPath.c_str());
        if (fAssociation.is_open())
        {
            while(!fAssociation.eof())
            {
                std::string s;
                getline(fAssociation,s);
                if(!s.empty())
                {
                    std::stringstream ss;
                    ss << s;
                    double t;
                    std::string sRGB;
                    ss >> t;
                    mvTimestamps.push_back(t);
                    ss >> sRGB;
                    mvstrImageFilenames.push_back(sRGB);
                }
            }
            fAssociation.close();
        }
        else
        {
            ROS_ERROR("Association file path wrong! Associate Path: %s", mstrAssociatedPath.c_str());
        }
    }
}