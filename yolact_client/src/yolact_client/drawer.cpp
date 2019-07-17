//
// Created by yonghui on 19-7-16.
//

#include "yolact_client/drawer.h"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <ros/ros.h>

namespace MY_SLAM
{
    float Drawer::colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
    
    
    Drawer::Drawer(const string &strClassNameFiles)
    {
        fstream fin(strClassNameFiles.c_str(), ios::in);
        if (fin.is_open())
        {
            string strClassName;
            while (getline(fin, strClassName))
            {
                mvstrClassName.push_back(strClassName);
            }
            fin.close();
        }
        else
        {
            ROS_ERROR("Read class name list fail!");
        }
    }
    
    
    // get yolo color
    cv::Scalar Drawer::GetColor(int nClassId, int nAllClasses)
    {
        int offset = nClassId*123457 % nAllClasses;
        float ratio = ((float)offset / nAllClasses) * 5;
        int i = floor(ratio);
        int j = ceil(ratio);
        ratio -= i;  // 相对基础色表的偏移
        float scB = (1 - ratio) * colors[i][0] + ratio * colors[j][0];
        float scG = (1 - ratio) * colors[i][1] + ratio * colors[j][1];
        float scR = (1 - ratio) * colors[i][2] + ratio * colors[j][2];
        return cv::Scalar(scB*255, scG*255, scR*255);
    }


    void Drawer::DrawMask(cv::Mat &im, const int &nClassId, const cv::Mat &imMask)
    {
//        assert(cv::countNonZero(imMask) < imMask.cols*imMask.rows/2);
        cv::Scalar clr = GetColor(nClassId);
        cv::Mat imClr = cv::Mat(im.rows, im.cols, CV_8UC3, clr);
        cv::Mat imClrWeight;
        cv::addWeighted(im, 0.55, imClr, 0.45, 0., imClrWeight);
        imClrWeight.copyTo(im, imMask);
    }


// draw bbox with labels
    void Drawer::DrawBoundingBox(cv::Mat &im, const int &nClassId, const float &fConfidence,
                         const int &nLeft, const int &nRight, const int &nUp, const int &nDown)
    {
        //! get label and color
        char cstrConfidence[7];
        sprintf(cstrConfidence, ", %4.2f", fConfidence);
        string strLabel;
        if (mvstrClassName.empty())
        {
            char cstrClassName[15];
            sprintf(cstrClassName, "Class_%02d", nClassId);
            strLabel = string(cstrClassName) + cstrConfidence;
        }
        else
        {
            strLabel = mvstrClassName[nClassId] + cstrConfidence;
        }
        cv::Scalar clr = GetColor(nClassId);
        
        //! create box
        cv::Point cv_ptLeftUp = cv::Point(nLeft, nUp);
        cv::Point cv_ptRightUp = cv::Point(nRight, nUp);
        cv::Point cv_ptRightDown = cv::Point(nRight, nDown);
        cv::Point cv_ptLeftDown = cv::Point(nLeft, nDown);
        cv::rectangle(im, cv_ptLeftUp, cv_ptRightDown, clr, 2);
        
        //! create label box
        int baseline = 0;
        cv::Size cv_sizeLabel = cv::getTextSize(strLabel, cv::FONT_HERSHEY_SIMPLEX, 0.6 , 2, &baseline);
        cv_sizeLabel.height = cv_sizeLabel.height + 10;  // boundary add
        cv_sizeLabel.width = cv_sizeLabel.width + 10;
        
        //! avoid label out of region
        int nLabelUp = (nUp-cv_sizeLabel.height>0) ? nUp - cv_sizeLabel.height : nUp;
        int nLabelLeft = (nLeft+cv_sizeLabel.width<im.cols) ? nLeft : im.cols-cv_sizeLabel.width;
        
        //! draw label and bounding box
        cv::Point cv_ptLableLeftUp = cv::Point(nLabelLeft, nLabelUp);
        cv::Point cv_ptLabelRightDown = cv::Point(nLabelLeft + cv_sizeLabel.width, nLabelUp + cv_sizeLabel.height);
        cv::Point cv_ptOrigin = cv::Point(nLabelLeft + 5, nLabelUp + cv_sizeLabel.height - 5);
        cv::rectangle(im, cv_ptLableLeftUp, cv_ptLabelRightDown, clr, -1);  // draw label box
        cv::putText(im, strLabel, cv_ptOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.6 , cv::Scalar(0, 0, 0), 2);  // draw text
    }
}
