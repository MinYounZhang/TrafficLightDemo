#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

#define DEBUG 0

typedef Eigen::RowVector4f CLS;
/*
 * 交通灯检测输出信息
 */
typedef struct TFLImageInfo
{
    std::string sensor;      // 关联传感器，如：“head_camera”
    double timestamp;        // 时间戳
    unsigned int frameCnt;   // 帧数
    unsigned int num = 0;    // 红绿灯数量
} TFLImageInfo;

typedef struct TFLPatchResult
{
    CLS scores;  // 置信度
    int netColor;      // 网络推理预测的颜色  0:green 1:unknow 2:red 3:yellow
    int modColor;      // 模版匹配预测的颜色  0:green 1:unmatch 2:red 3:yellow
    int countColor;    // 像素点计数预测的颜色
    int fusionColor;    // 最终输出颜色
    int trackId;       // 跟踪算法 轨迹id

    float longitudeDis;  // 纵向距离
    float latitudeDis;   // 横向距离
    float detConf;     // 检测置信度
    std::vector<float> tlbr;  // 坐标信息,上左下右点坐标
    std::vector<float> xywh;  // 坐标信息,XY中心+宽高
} TFLPatchResult;


typedef struct TFLtrack
{
    uint type;      // color
    uint id;        // track id 
    uint left;  
    uint top;
    uint right;
    uint bottom;
    float score;    // using det conf now
    float longitudeDis;  // 纵向距离
    float latitudeDis;   // 横向距离
    float heigthDis;
} TFLtrack;


typedef struct TFLResult
{
    TFLImageInfo tflImageInfo;   // 交通灯图片的信息（相机序号、时间戳、帧数序列号、红绿灯的数量）
    std::vector<TFLPatchResult> patchsResult;       // 一张图包含多个交通灯
    std::vector<TFLtrack> tracks;
} TFLResult;

// 测距所需要的相机内外参
typedef struct DisInit
{
    float h;                    // 相机离地面距离
    float pitch;                // 俯仰角
    std::vector<double> mtx;    // 内参矩阵
    std::vector<double> dist;   // 畸变系数
    std::vector<double> r;      // 相机外参，相对于车体 旋转矩阵
    std::vector<double> t;      // 相机外参，相对于车体 平移矩阵
}DisInit;
