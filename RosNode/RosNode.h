#pragma once
#include <map>
#include <memory>
#include "Common.h"
#include <ros/ros.h>
#include "ParseArgs.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include "json/json.h"
#include "DetectObjectInfo.h"
#include "message/TrafficLights.h"
#include "message/Dets.h"
class RosNode;
class TrafficLightInterface;
struct TFLResult;
class RosNode
{
public:
    // 启动ros节点
    void Run(std::shared_ptr<ParseArgs> args);

private:

    void DetecteCallback(const autocity::DetsPtr &message);

    bool Progress(const sensor_msgs::ImagePtr &message);
    bool Progress(const autocity::DetsPtr &message);

    //检测节点返回的json转换为box数据
    bool JsonToDetecteObject(std::string json,TrafficBoxList &boxs);
    //检测节点转box数据
    bool DetsToDetecteObject(const autocity::DetsPtr &data,cv::Mat &image,TrafficBoxList &boxes);

    autocity::TrafficLights ResultToMsg(std::shared_ptr<TFLResult> result);

    void PublishResult();
    //void PublishShow(const sensor_msgs::ImagePtr &message);
    void PublishShow(const autocity::DetsPtr &message);

private:
    std::string                                 mClassName = "RosNode";
    std::string                                 mCameraName;
    std::shared_ptr<TrafficLightInterface>      mInterface;
    std::string                                 mTrafficLightLabel = "trafficLight";

    std::string                                 mSensorYaml = "config/sensor_param.yaml";
    std::string                                 mPerceptionYaml = "config/perception.yaml";

    ros::NodeHandle                             mRosHandle;         // ros handle
    std::string                                 mImageSubscribeTopic;
    std::string                                 mDetectSubscribeTopic;
    std::string                                 mPublishTopic;

    ros::Subscriber                             mImageSubscriber;
    ros::Subscriber                             mDetecteSubscriber;
    ros::Publisher                              mPublisher;
    ros::Publisher                              mShowPublisher;
    bool                                        mShow = false;
};
