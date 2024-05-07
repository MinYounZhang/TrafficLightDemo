#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "Log.h"
//#include "loguru.h"
#include <opencv2/imgproc/types_c.h> 
#include "TrafficLightInfo.h"


//ImgPipline
#include "NvCaffeParser.h"
#include "NvInfer.h"
#include "cuda_check.h"


#define TFL_GREEN 0
#define TFL_OFF 1
#define TFL_RED 2
#define TFL_YELLOW 3

#define THREE_ONE 0
#define ONE_THREE 1
#define NO_MATCH 2

#define IMAGE_SIZE 32

/*
 * 红绿灯检测
 */
class TrafficLightClssify
{
public:
    TrafficLightClssify();
    ~TrafficLightClssify();

    // 初始化
    void InitModel(const std::string &modelPath);

    // 反初始化
    void UninitModel();

    // 交通灯检测
    void Classify(const cv::Mat &image, std::shared_ptr<TFLResult> tflResult);

    void static ColorFusion(TFLPatchResult &tflpatchResult);

private:
    nvinfer1::IExecutionContext* context = NULL;
	nvinfer1::ICudaEngine* engine = NULL;
	nvinfer1::IRuntime* runtime = NULL;
    bool usingMod = true;

    struct ModConfig
    {
        float cols_rate = 0.333;
        float rows_rate = 3;
    } modConfig;
    int PixelNum = 30;
    
private:
    // 交通灯颜色检测(卷积网络推理)
    void DNNClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult);
    // 交通灯颜色检测(像素点计数)
    void ColorClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult);
    // 交通灯颜色检测(模版匹配)
    unsigned int TemplateMatch(int cols, int rows);
    void ModClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult);
    	
    // 加载engine模型参数
    nvinfer1::IExecutionContext* buildEngine(std::string engineName);

    // 加载engine模型参数,使用tensorrt进行推理
    void doInference(nvinfer1::IExecutionContext& context, void* input, void* output, int batchSize, size_t inputSize, size_t outputSize);
};