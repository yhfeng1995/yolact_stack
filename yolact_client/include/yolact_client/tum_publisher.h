//
// Created by yonghui on 19-7-16.
//

#ifndef YOLACT_CLIENT_TUM_PUBLISHER_H
#define YOLACT_CLIENT_TUM_PUBLISHER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>
using namespace std;

namespace MY_SLAM
{
    class TumPublisher
    {
    public:
        TumPublisher(ros::NodeHandle &nh);
        bool PublishNext();
        
    protected:
        void LoadImagesRGBTUM();
        
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher pub_;
    
        // publish tum use
        int mnCurrentIdx;
        std::string mstrDataSetDir;
        std::string mstrSeqName;
        std::string mstrAssociatedPath;
        std::vector<std::string> mvstrImageFilenames;
        std::vector<double> mvTimestamps;
    };
}

#endif //YOLACT_CLIENT_TUM_PUBLISHER_H
