#pragma once
#include <map>
#include <mutex>
#include <time.h>
#include <iostream>

// 功能ID
#define GROUP_PERCEPTION 0

// 错误等级
#define WARN_LEVEL  0
#define ERROR_LEVEL 1
#define FATAL_LEVEL 2

/*
 * 各个模块的定义
 */
#define CAMERA_DETECTE_MODULE_ID 0      // 图像检测
#define CAMERA_TRACKING_MODULE_ID 1     // 图像跟踪
#define LIDAR_MERGE_MODULE_ID 2         // 点云合并
#define LIDAR_CLUSTER_MODULE_ID 3       // 点云聚类
#define LIDAR_DETECTE_MODULE_ID 4       // 点云检测
#define LIDAR_TRACKING_MODULE_ID 5      // 点云跟踪
#define CURB_DETECTE_MODULE_ID 6        // 路沿检测
#define TRAFFIC_LIGHT_MODULE_ID 7       // 红绿灯检测
#define OBJECT_FUSTION_MODULE_ID 8      // 目标融合

// 当前的模块
#define MODULE_ID TRAFFIC_LIGHT_MODULE_ID

/*
 * 子模块，开发者自行定义
 */
#define SUB_MODEL_TRAFFIC_LIGHT 0

// 起始错误码
#define WARN_LOG_ID ((WARN_LEVEL << 28) | (GROUP_PERCEPTION << 18) |(MODULE_ID << 12) | (SUB_MODEL_TRAFFIC_LIGHT << 8))
#define ERROR_LOG_ID ((ERROR_LEVEL << 28) | (GROUP_PERCEPTION << 18) |(MODULE_ID << 12) | (SUB_MODEL_TRAFFIC_LIGHT << 8))
#define FATAL_LOG_ID ((FATAL_LEVEL << 28) | (GROUP_PERCEPTION << 18) |(MODULE_ID << 12) | (SUB_MODEL_TRAFFIC_LIGHT << 8))

/*
 * 日志相关
 */
#ifndef LOG4CPLUS

#define LOG_V(info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << std::endl;}
#define LOG_D(info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << std::endl;}
#define LOG_I(info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << std::endl;}
#define LOG_W(code, info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << ", error code:" << code << std::endl;}
#define LOG_E(code, info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << ", error code:" << code << std::endl;}
#define LOG_F(code, info)  {std::stringstream tmpInfo; tmpInfo << info;std::cout <<tmpInfo.str() << ", error code:" << code << std::endl;}

#else

bool isRepeat(unsigned int code);

extern void writeLog(const std::string &info, int logLevel);
#define LOG_V(info)  {std::stringstream tmpInfo; tmpInfo << info;writeLog(tmpInfo.str(), common_log::Log_Level_Verbose);}
#define LOG_D(info)  {std::stringstream tmpInfo; tmpInfo << info;writeLog(tmpInfo.str(), common_log::Log_Level_Debug);}
#define LOG_I(info)  {std::stringstream tmpInfo; tmpInfo << info;writeLog(tmpInfo.str(), common_log::Log_Level_Info);}
#define LOG_W(code, info)  {if (false == isRepeat(code)) {std::stringstream tmpInfo; tmpInfo << info << ", error code:" << code;writeLog(tmpInfo.str(), 30);}}
#define LOG_E(code, info)  {if (false == isRepeat(code)) {std::stringstream tmpInfo; tmpInfo << info << ", error code:" << code;writeLog(tmpInfo.str(), 40);}}
#define LOG_F(code, info)  {if (false == isRepeat(code)) {std::stringstream tmpInfo; tmpInfo << info << ", error code:" << code;writeLog(tmpInfo.str(), 50);}}

#endif
