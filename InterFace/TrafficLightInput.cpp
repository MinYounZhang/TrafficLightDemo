#include <sstream>
#include "TrafficLightInput.h"

// 设置mat
void TrafficLightInput::SetImage(cv::Mat image)
{
    mImage = image.clone();
}
cv::Mat TrafficLightInput::GetImage()
{
    return mImage;
}
void TrafficLightInput::SetBoxes(TrafficBoxList boxs)
{
    mBoxs = boxs;
}
TrafficBoxList TrafficLightInput::GetBoxes()
{
    return mBoxs;
}
std::string TrafficLightInput::ToString()
{
    std::stringstream buf;
    return buf.str();
}
