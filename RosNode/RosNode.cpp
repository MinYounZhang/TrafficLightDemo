#include "RosNode.h"
#include "Configure.h"
#include "json/json.h"
#include "TrafficLightInput.h"
#include "TrafficLightOutput.h"
#include "TrafficLightInterface.h"
#include "Common/Common.h"
#include "Base.h"
#include "StatusReporterMng.h"
// 启动ros节点
void RosNode::Run(std::shared_ptr<ParseArgs> args)
{
    mInterface = std::make_shared<TrafficLightInterface>();

    // 加载参数
    std::shared_ptr<Configure> parm = std::make_shared<Configure>();
    args->GetParam("show", mShow); 
    parm->Init(mSensorYaml, mPerceptionYaml, args);
    YAML::Node sensorConfig = *parm->GetSensorYaml();
    YAML::Node perceptionConfig = *parm->GetPerceptionYaml();
    mInterface->Init(parm);
    mDetectSubscribeTopic = perceptionConfig["camera"]["detect"]["publish_topic"].as<std::string>();
    mPublishTopic = perceptionConfig["camera"]["traffic_light"]["publish_topic"].as<std::string>();
    mCameraName = perceptionConfig["camera"]["detect"]["subscribe_topic"]["front"].as<std::string>();
    mDetecteSubscriber = mRosHandle.subscribe(mDetectSubscribeTopic, 1, &RosNode::DetecteCallback, this);

    mPublisher = mRosHandle.advertise<autocity::TrafficLights>(mPublishTopic, 1);
    std::string showTopic = mPublishTopic + "/show";
    mShowPublisher = mRosHandle.advertise<sensor_msgs::Image>(showTopic, 1);

    //状态上报
    STATUS_REPORTER_MNG->SetNodeName("TrafficLight");
    STATUS_REPORTER_MNG->SetHeartBeatCode(ERROR_LOG_ID);
    STATUS_REPORTER_MNG->Init();
    STATUS_REPORTER_MNG->StartHeartBeat();

    // 进入ros消息循环
    ros::spin();

    //mImageSubscriber.shutdown();
    mDetecteSubscriber.shutdown();
    mPublisher.shutdown();
    mShowPublisher.shutdown();
   // 释放资源
    mInterface->Uninit();
    STATUS_REPORTER_MNG->DeInit();
}


void RosNode::DetecteCallback(const autocity::DetsPtr &message)
{
    if(Progress(message))
    {
        PublishResult();
        PublishShow(message);
    }
}
bool RosNode::Progress(const sensor_msgs::ImagePtr &message)
{
    STATUS_REPORTER_MNG->StartProgress(message->header.seq);
    TrafficBoxList boxs;
    bool findTrafficLight = JsonToDetecteObject(message->header.frame_id,boxs);
    if(false==findTrafficLight)
    {
        STATUS_REPORTER_MNG->EndProgress();
        return false;
    }
    cv_bridge::CvImagePtr messageImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8); 
    cv::Mat image= messageImage->image.clone();
    std::shared_ptr<TrafficLightInput> input = std::make_shared<TrafficLightInput>();
    input->SetImage(image);
    input->SetBoxes(boxs);
    mInterface->UpdateInput(input);
    mInterface->Run();
    STATUS_REPORTER_MNG->EndProgress();
    return true;
}
bool RosNode::Progress(const autocity::DetsPtr &message)
{
    STATUS_REPORTER_MNG->StartProgress(message->header.frameCnt);
    TrafficBoxList boxs;
    cv::Mat image;
    bool findTrafficLight = DetsToDetecteObject(message,image,boxs);
    if(false==findTrafficLight)
    {
        STATUS_REPORTER_MNG->EndProgress();
        return false;
    }
    std::shared_ptr<TrafficLightInput> input = std::make_shared<TrafficLightInput>();
    input->SetImage(image);
    input->SetBoxes(boxs);
    mInterface->UpdateInput(input);
    mInterface->Run();
    STATUS_REPORTER_MNG->EndProgress();
    return true;
}
bool RosNode::JsonToDetecteObject(std::string json,TrafficBoxList &boxs)
{
    Json::Reader reader;
    Json::Value readerRoot;
    bool ret = reader.parse(json, readerRoot);
    if (false == ret)
    {
        return false;
    }
    boxs.boxes.clear();
    for (int i = 0; i < readerRoot["object"].size(); ++i)
    {
        int type = readerRoot["object"][i]["type"].asInt();
        if(type==Common::GetCameraObjectTypeByName(mTrafficLightLabel))
        {
            int top,bottom,left,right;
            float sc;
            left = readerRoot["object"][i]["left"].asFloat();
            top = readerRoot["object"][i]["top"].asFloat();
            right = readerRoot["object"][i]["right"].asFloat();
            bottom = readerRoot["object"][i]["bottom"].asFloat();
            sc = readerRoot["object"][i]["score"].asFloat();
            TrafficBox tb;
            tb.left = left;
            tb.top = top;
            tb.right = right;
            tb.bottom = bottom;
            tb.score = sc;
            boxs.boxes.push_back(tb);
        }
    }
    if(boxs.boxes.size()==0)
    {
        return false;
    }else{
        return true;
    }
    
    
}

bool RosNode::DetsToDetecteObject(const autocity::DetsPtr &data,cv::Mat &image,TrafficBoxList &boxes)
{
    bool find = false;
    for(int i=0;i<data->dets.size();i++)
    {
        if(data->dets[i].camera==mCameraName)
        {
            for(int j=0;j<data->dets[i].boxes.size();j++)
            {
                int type = data->dets[i].boxes[j].type;
                if(type==Common::GetCameraObjectTypeByName(mTrafficLightLabel))
                {
                    TrafficBox tb;
                    tb.left = data->dets[i].boxes[j].left;
                    tb.top = data->dets[i].boxes[j].top;
                    tb.right = data->dets[i].boxes[j].right;
                    tb.bottom = data->dets[i].boxes[j].bottom;
                    tb.score = data->dets[i].boxes[j].score;
                    boxes.boxes.push_back(tb);
                    find = true;
                }
            }
            image = cv::Mat(data->dets[i].height,data->dets[i].width,CV_8UC3,data->dets[i].image.data()).clone();
        }
    }
    return find;
}
autocity::TrafficLights RosNode::ResultToMsg(std::shared_ptr<TFLResult> result)
{
    autocity::TrafficLights output;
    output.header.frameCnt = result->tflImageInfo.frameCnt;
    output.header.timestamp = result->tflImageInfo.timestamp;
    output.header.sensor = result->tflImageInfo.sensor;
    output.header.num = result->tflImageInfo.num;
    for(int i=0;i<result->tracks.size();i++)
    {
        autocity::TrafficLight light;
        light.type = result->tracks[i].type;
        light.id = result->tracks[i].id;
        light.score = result->tracks[i].score;
        light.longitudeDis = result->tracks[i].longitudeDis;
        light.latitudeDis = result->tracks[i].latitudeDis;
        light.heigthDis = result->tracks[i].heigthDis;
        light.reserved.push_back(result->tracks[i].left);
        light.reserved.push_back(result->tracks[i].top);
        light.reserved.push_back(result->tracks[i].right);
        light.reserved.push_back(result->tracks[i].bottom);

        output.trafficLights.push_back(light);
    }
    return output;
}
void RosNode::PublishResult()
{
    std::pair<std::shared_ptr<OutputInterface>, std::shared_ptr<Error>> result;
    result = mInterface->GetResult();
    std::shared_ptr<TFLResult> obj = std::dynamic_pointer_cast<TrafficLightOutput>(result.first)->GetObject();
    autocity::TrafficLights trafficLights = ResultToMsg(obj);
    mPublisher.publish(trafficLights);
}

void RosNode::PublishShow(const autocity::DetsPtr &message)
{
    if(false==mShow)
    {
        return;
    }

    cv::Scalar scalar = cv::Scalar(255,255,255);
    cv::Mat showImage;
    for(int i=0;i<message->dets.size();i++)
    {
        if(message->dets[i].camera==mCameraName)
        {
            showImage = cv::Mat(message->dets[i].height,message->dets[i].width,CV_8UC3,message->dets[i].image.data());
        }
    }
    std::vector<std::string> trafficLightText = {"green","unknow","red","yellow"};
    std::pair<std::shared_ptr<OutputInterface>, std::shared_ptr<Error>> result;
    result = mInterface->GetResult();
    std::shared_ptr<TFLResult> obj = std::dynamic_pointer_cast<TrafficLightOutput>(result.first)->GetObject();
    
    for(int i=0;i<obj->tracks.size();i++)
    {
        // 绘制矩形框
        int left = obj->tracks[i].left;
        int top = obj->tracks[i].top;
        int right = obj->tracks[i].right;
        int bottom = obj->tracks[i].bottom;
        int type = obj->tracks[i].type;
        std::stringstream xShow, yShow;
        xShow.precision(1); yShow.precision(1);   // 保留一位小数
        xShow.setf(std::ios::fixed); yShow.setf(std::ios::fixed);
        xShow << obj->tracks[i].longitudeDis; yShow << obj->tracks[i].latitudeDis;

        if(type<trafficLightText.size())
        {
            cv::putText(showImage, trafficLightText[type], cv::Point(left, top - 10), 2, 1, cv::Scalar(0, 0, 255));
            switch (type){
                case 0:
                    scalar = cv::Scalar(0, 255, 0);
                    break;
                case 1:
                    scalar = cv::Scalar(0, 0, 0);
                    break;
                case 2:
                    scalar = cv::Scalar(0, 0, 255);
                    break;
                case 3:
                    scalar = cv::Scalar(255, 255, 0);
                    break;
            }
        }
        cv::rectangle(showImage, cv::Rect(left, top, right-left, bottom-top), scalar, 2);
        cv::putText(showImage, trafficLightText[type], cv::Point(left, top - 10), 2, 1, cv::Scalar(0, 0, 255));
        // cv::putText(showImage, std::to_string(obj->tracks[i].id), cv::Point(left, top + 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2);
        // cv::putText(showImage, xShow.str(), cv::Point(left, top), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2);
        // cv::putText(showImage, yShow.str(), cv::Point(left + (right - left) / 2, top), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 2);
        
    }
    std_msgs::Header header;
    header.frame_id = "autocity";
    sensor_msgs::ImagePtr showMessage = cv_bridge::CvImage(header, "bgr8", showImage).toImageMsg();
    mShowPublisher.publish(showMessage);
}