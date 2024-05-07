#pragma once
#include "InputInterface.h"
#include "opencv2/opencv.hpp"
#include "DetectObjectInfo.h"
/*
 * 算法的输入
 */

class TrafficLightInput : public InputInterface
{
public:

    // 设置mat
    void SetImage(cv::Mat image);
    cv::Mat GetImage();
    void SetBoxes(TrafficBoxList boxs);
    TrafficBoxList GetBoxes();

    virtual std::string ToString() override;
private:
    cv::Mat                         mImage;
    TrafficBoxList                  mBoxs;
};
