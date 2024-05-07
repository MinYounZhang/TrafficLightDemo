#pragma once
#include <fstream>
#include <iostream>
#include <sstream>

#include "NvInfer.h"
#include "NvOnnxParser.h"



/*
 * GPU上创建推理engine比较耗时，所以在程序初始化的时候创建推理Engine
 */
class NetEngine
{
public:
	int onnx2engine(std::string onnx_filename, std::string enginefilePath, int type);
private:
	nvinfer1::IBuilder              *mBuilder = NULL;
	nvinfer1::INetworkDefinition    *mNetwork = NULL;
	nvonnxparser::IParser           *mParser = NULL;
};
