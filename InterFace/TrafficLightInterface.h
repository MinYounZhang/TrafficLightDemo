#pragma once
#include <memory>
#include "TrafficLightOutput.h"
#include "PerceptionInterface.h"
#include "TrafficLight.h"
#include "BYTETracker.h"
#include "DistanceTrafficLight.h"
#include "Log.h"

class TrafficLightInterface : public PerceptionInterface
{
public:
    TrafficLightInterface();
    virtual ~TrafficLightInterface();
    // 初始化
    virtual std::shared_ptr<Error> Init(std::shared_ptr<Configure> parm) override;

    // 反初始化
    virtual void Uninit() override;

    // 更新配置参数
    virtual std::shared_ptr<Error> UpdateParm(std::shared_ptr<Configure> parm) override;

    // 设置输入
    virtual std::shared_ptr<Error> UpdateInput(std::shared_ptr<InputInterface> input) override;

    // 处理输入数据
    virtual std::shared_ptr<Error> Run() override;

    // 获取输出结果
    virtual std::pair<std::shared_ptr<OutputInterface>, std::shared_ptr<Error>> GetResult() override;
private:

    std::shared_ptr<InputInterface>             mInputData;     // 输入数据
    std::shared_ptr<TrafficLightOutput>         mOutput;        // 目标检测的结果

    std::shared_ptr<TrafficLightClssify>        mTrafficLightClassify;//红绿灯检测
    std::shared_ptr<BYTETracker>                mTrafficLightTracker;       // 红绿灯跟踪    // ming
    std::shared_ptr<DistanceTrafficLight>       mDistanceTrafficLight;      // 红绿灯测距
};
