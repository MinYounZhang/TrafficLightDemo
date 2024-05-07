#include "NetEngine.h"

// 全局创建 ILogger 类型的对象
class Logger :public nvinfer1::ILogger
{
public:
	virtual void log(Severity severity, const char* msg) noexcept override
	{
		// suppress info-level messages
		if (severity != Severity::kINFO) {
			std::cout << msg << std::endl;
		}
	}
} HLogger;

int NetEngine::onnx2engine(std::string onnx_filename, std::string enginefilePath, int type) {


	// 创建builder
	mBuilder = nvinfer1::createInferBuilder(HLogger);

	// 创建network
	const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
	mNetwork = mBuilder->createNetworkV2(explicitBatch);

	// 创建onnx模型解析器
	mParser = nvonnxparser::createParser(*mNetwork, HLogger);

	// 解析模型
	mParser->parseFromFile(onnx_filename.c_str(), 2);
	for (int i = 0; i < mParser->getNbErrors(); ++i)
	{
		std::cout << mParser->getError(i)->desc() << std::endl;
	}
	printf("tensorRT load onnx model sucessful! \n"); // 解析模型成功，第一个断点测试


	// 使用builder对象构建engine
	nvinfer1::IBuilderConfig *config = mBuilder->createBuilderConfig();
	config->setMaxWorkspaceSize(1 << 28);  // 设置最大工作空间

	nvinfer1::ICudaEngine* myengine = mBuilder->buildEngineWithConfig(*mNetwork, *config); //创建engine  第二个断点测试
	std::cout << "try to save engine file now" << std::endl;
	std::ofstream p(enginefilePath, std::ios::binary);
	if (!p) {
		std::cerr << "could not open plan output file" << std::endl;
		return 0;
	}

	// 序列化
	nvinfer1::IHostMemory *modelStream = myengine->serialize();
	p.write(reinterpret_cast<const char*>(modelStream->data()), modelStream->size()); // 写入

	modelStream->destroy(); // 销毁
	myengine->destroy();
	mNetwork->destroy();
	mParser->destroy();
	std::cout << "convert onnx model to TensorRT engine model successfully!" << std::endl; // 转换成功，第四个断点测试

	return 0;
}