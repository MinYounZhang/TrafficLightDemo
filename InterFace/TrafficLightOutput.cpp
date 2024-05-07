#include <sstream>
#include "TrafficLightOutput.h"

// 检测出来的目标
void TrafficLightOutput::SetObject(std::shared_ptr<TFLResult> objects)
{
    mObject = objects;
}
std::shared_ptr<TFLResult> TrafficLightOutput::GetObject()
{
    return mObject;
}

std::string TrafficLightOutput::ToString()
{
    std::stringstream buf;
    return buf.str();
}
