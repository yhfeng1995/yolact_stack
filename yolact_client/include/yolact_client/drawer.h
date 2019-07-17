//
// Created by yonghui on 19-7-16.
//

#ifndef YOLACT_CLIENT_DRAWER_H
#define YOLACT_CLIENT_DRAWER_H

#include <opencv2/core.hpp>
#include <string>
using namespace std;

namespace MY_SLAM
{
    class Drawer
    {
    public:
        static float colors[6][3];
        
        Drawer(){};
        
        Drawer(const string &strClassNameFiles);
        
        cv::Scalar GetColor(int nClassId, int nAllClasses=80);
        void DrawMask(cv::Mat &im, const int &nClassId, const cv::Mat &imMask);
        void DrawBoundingBox(cv::Mat &im, const int &nClassId, const float &fConfidence,
                             const int &nLeft, const int &nRight, const int &nUp, const int &nDown);

    protected:
        vector<string> mvstrClassName;
    };
}


#endif //YOLACT_CLIENT_DRAWER_H
