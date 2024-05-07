#include "Common.h"
#include "TrafficLightInput.h"
#include "TrafficLightInterface.h"

TrafficLightInterface::TrafficLightInterface()
{
    mTrafficLightClassify = std::make_shared<TrafficLightClssify>();

    int fps=20;
    mTrafficLightTracker = std::make_shared<BYTETracker>(fps, 30);   
}

TrafficLightInterface::~TrafficLightInterface()
{

}
std::shared_ptr<Error> TrafficLightInterface::Init(std::shared_ptr<Configure> parm)
{
    YAML::Node perceptionConfig = *parm->GetPerceptionYaml();
    YAML::Node sensorConfig = *parm->GetSensorYaml();

    std::shared_ptr<DisInit> param = std::make_shared<DisInit>();
    param->h = sensorConfig["camera"]["front_center"]["height"].as<float>();
    param->pitch = sensorConfig["camera"]["front_center"]["pitch"].as<float>();
    param->mtx = sensorConfig["camera"]["front_center"]["K"].as<std::vector<double>>();
    param->dist = sensorConfig["camera"]["front_center"]["D"].as<std::vector<double>>();
    param->r = sensorConfig["camera"]["front_center"]["rotation"].as<std::vector<double>>();
    param->t = sensorConfig["camera"]["front_center"]["translation"].as<std::vector<double>>();

    // 初始化
    std::string modelPath = perceptionConfig["camera"]["traffic_light"]["model"].as<std::string>();
    mTrafficLightClassify->InitModel(modelPath);
    return Common::NoError();
}
void TrafficLightInterface::Uninit()
{

}
std::shared_ptr<Error> TrafficLightInterface::UpdateParm(std::shared_ptr<Configure> parm)
{
    return Common::NoError();
}

std::shared_ptr<Error> TrafficLightInterface::UpdateInput(std::shared_ptr<InputInterface> input)
{
    mInputData = input;
    return Common::NoError();
}

std::shared_ptr<Error> TrafficLightInterface::Run()
{
    cv::Mat image = std::dynamic_pointer_cast<TrafficLightInput>(mInputData)->GetImage();
    TrafficBoxList boxList = std::dynamic_pointer_cast<TrafficLightInput>(mInputData)->GetBoxes();
    std::shared_ptr<TFLResult> tflResult = std::make_shared<TFLResult>();
    for(int i=0;i<boxList.boxes.size();i++)
    {
        TFLPatchResult patch;
        int width = (boxList.boxes[i].right-boxList.boxes[i].left);
        int height = (boxList.boxes[i].bottom-boxList.boxes[i].top);
        patch.tlbr = {  (float)boxList.boxes[i].top,
                        (float)boxList.boxes[i].left, 
                        (float)boxList.boxes[i].bottom,
                        (float)boxList.boxes[i].right};
        patch.xywh = {  (float)(boxList.boxes[i].left+width/2.),
                        (float)(boxList.boxes[i].top+height/2.),
                        (float)width,
                        (float)height};
        patch.detConf = (float)boxList.boxes[i].score;
        patch.netColor = 999;
        tflResult->patchsResult.emplace_back(patch);
    }
    
    mTrafficLightClassify->Classify(image, tflResult);
    mTrafficLightTracker->update(tflResult->patchsResult, image, tflResult->tracks);
    
    // polytracks
    mOutput = std::make_shared<TrafficLightOutput>();
    mOutput->SetObject(tflResult);
    return Common::NoError();
}

std::pair<std::shared_ptr<OutputInterface>, std::shared_ptr<Error>> TrafficLightInterface::GetResult()
{
    std::pair<std::shared_ptr<OutputInterface>, std::shared_ptr<Error>> result(mOutput, Common::NoError());
    return result;
}