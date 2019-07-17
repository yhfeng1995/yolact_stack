//
// Created by yonghui on 19-7-16.
//

#include "yolact_client/yolact_client.h"
#include "semantic_msgs/InstanceList.h"
#include "semantic_msgs/Instance.h"
#include "semantic_msgs/RunInstance.h"
#include <fstream>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "maskApi.h"


namespace MY_SLAM
{
    YolactClient::YolactClient(ros::NodeHandle &nh) :
    nh_(nh), it_(nh)
    {
        private_nh_ = ros::NodeHandle("~");
        
        // [optional] publish visualization
        private_nh_.getParam("visual", mbVisual);
        if (mbVisual)
        {
            pub_visual_ = it_.advertise("image_visual", 1);
            std::string strClassNameFile;
            private_nh_.getParam("class_name_path", strClassNameFile);
            mUtils = Drawer(strClassNameFile);
        }
    
        // TODO: create new network msg and srv, change callback and client
        // publish result
        pub_result_ = nh_.advertise<semantic_msgs::InstanceList>("result", 1);
        // subscriber bind callback
        sub_ = nh_.subscribe("image/compressed", 1, &YolactClient::YolactMaskCb, this);
        // run network client
        client_ = nh_.serviceClient<semantic_msgs::RunInstance>("run_yolact");
    }
    
    
    void YolactClient::YolactMaskCb(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        ros::Time tBegin = ros::Time::now();
        cv::Mat imRGB;
        // get image
        try
        {
            imRGB = cv::imdecode(msg->data, 1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("could not convert image: %s", e.what());
        }

        // TODO: Change call service and response type
        // build up server message
        semantic_msgs::RunInstance srv;
        semantic_msgs::InstanceList msgResultList;
        vector<semantic_msgs::Instance> vResults;
        srv.request.image = *msg;
        
        // wait server...
        if (client_.call(srv))
        {
            msgResultList = srv.response.result;
            vResults = msgResultList.instances;
        }

        ros::Time tEnd = ros::Time::now();
        ROS_INFO("Frame: %17.6f; Cost Time: %6.4f", msg->header.stamp.toSec(), (tEnd-tBegin).toSec());
        
        // draw result and publish
        if (mbVisual)
        {
            for (int i=0; i<vResults.size(); i++)
            {
                // draw mask
                unsigned char *mpMaskData = new uchar[imRGB.rows*imRGB.cols];
                COCOAPI::RLE *r = new COCOAPI::RLE();
                COCOAPI::rleFrString(r, vResults[i].segmentation.counts.c_str(), imRGB.rows, imRGB.cols);
                COCOAPI::rleDecode(r, mpMaskData, 1);
                COCOAPI::rleFree(r);  // rle decode complete
                cv::Mat imMask = cv::Mat(imRGB.cols, imRGB.rows, CV_8UC1, mpMaskData);
                cv::transpose(imMask, imMask);
                mUtils.DrawMask(imRGB, vResults[i].category_id, imMask);
                // convert bbox represent
                int nLeft = vResults[i].bbox.b[0];
                int nRight = vResults[i].bbox.b[0] + vResults[i].bbox.b[2];
                int nUp = vResults[i].bbox.b[1];
                int nDown = vResults[i].bbox.b[1] + vResults[i].bbox.b[3];
                // draw bbox
                mUtils.DrawBoundingBox(imRGB, vResults[i].category_id, vResults[i].score, nLeft, nRight, nUp, nDown);
            }
            std_msgs::Header h;
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(h, sensor_msgs::image_encodings::BGR8, imRGB));
            pub_visual_.publish(cv_ptr->toImageMsg());
        }
        pub_result_.publish(srv.response.result);
    }
}