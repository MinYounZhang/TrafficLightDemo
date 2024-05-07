#pragma once
#include <string>
#include <vector>

/*
 * 目标检测信息(只包含交通灯信息)
 */
typedef struct TrafficBox
{
	float left;               //左上角坐标x
	float top;                //左上角坐标y
	float right;              //右下角坐标x（已复原到原图坐标）
	float bottom;             //右下角坐标y
	float score;              //score = ObjConf * ClsConf
}TrafficBox;

typedef struct TrafficBoxList //输出信息的结构体
{
	std::vector<TrafficBox> boxes;
}TrafficBoxList;