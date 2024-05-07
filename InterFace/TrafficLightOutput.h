#pragma once
#include <memory>
#include "OutputInterface.h"
#include "TrafficLightInfo.h"
/*
 * 算法的输出
 */
class TrafficLightOutput : public OutputInterface
{
public:
    // 检测出来的目标
    void SetObject(std::shared_ptr<TFLResult> objects);
    std::shared_ptr<TFLResult> GetObject();

    virtual std::string ToString() override;
private:
    std::shared_ptr<TFLResult>    mObject;
};
