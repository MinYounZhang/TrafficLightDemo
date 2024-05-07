#include "TrafficLight.h"

// #define DEBUG
#define WRITE_IMAGE


const cv::Scalar IMG_MEAN(123.675, 116.28, 103.53);
const cv::Scalar IMG_STD(58.395, 57.12, 57.375);
constexpr auto INPUT_NAME = "input"; // engine输入头的名称
constexpr auto OUTPUT_NAME = "output";// engine输出头的名称
std::map<int, int> TFL_COLOR_MAP = {{0, TFL_RED}, {1, TFL_YELLOW}, {2, TFL_GREEN}};

// template <typename iterable>
// inline iterable softmax(iterable value)
// {
// }

// 日志文件类创建
class Logger : public nvinfer1::ILogger
{
public:
	virtual void log(Severity severity, const char* msg) noexcept override
	{
		if (severity <= Severity::kINFO) {
			std::cout << msg << std::endl;
		}
	}
} gLogger;

TrafficLightClssify::TrafficLightClssify()
{
}

TrafficLightClssify::~TrafficLightClssify()
{
}

// 模型初始化
void TrafficLightClssify::InitModel(const std::string& Path)
{
	context = buildEngine(Path);
}

// 反初始化
void TrafficLightClssify::UninitModel()
{
	// TODO
}


// 交通灯检测
void TrafficLightClssify::Classify(const cv::Mat &image, std::shared_ptr<TFLResult> tflResult)
{
	for(auto i = 0; i < tflResult->patchsResult.size(); i++){ // 交通灯颜色检测，可能有多个红绿灯
		auto &patchResult =  tflResult->patchsResult.at(i);
		cv::Mat roi = image(cv::Rect(
									patchResult.tlbr[1], \
									patchResult.tlbr[0], \
									patchResult.xywh[2], \
									patchResult.xywh[3])
		);
		this->DNNClassify(roi, patchResult); // step 1: 模型推理颜色
		if(this->usingMod && patchResult.scores( patchResult.netColor ) < 0.6)
		{
			this->ModClassify(roi, patchResult); // step 2: 图像处理识别颜色
			this->ColorClassify(roi, patchResult); // step 3: 图片像素颜色计数
			TrafficLightClssify::ColorFusion(patchResult); // step 4: 颜色聚合
		}
		else
		{
			patchResult.fusionColor = patchResult.netColor;
		}
		// 
	}
}


// 交通灯颜色检测
void TrafficLightClssify::ColorClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult)
{
	cv::Mat hsv;
	cvtColor(img, hsv, cv::ColorConversionCodes::COLOR_BGR2HSV); // BGR转成HSV

	cv::Scalar lower_red1 = cv::Scalar(160, 100, 100); // 设置红、黄、绿的颜色范围阈值
	cv::Scalar upper_red1 = cv::Scalar(180, 255, 255);
	cv::Scalar lower_red2 = cv::Scalar(35, 100, 100);
	cv::Scalar upper_red2 = cv::Scalar(10, 255, 255);
	cv::Scalar lower_green = cv::Scalar(78, 43, 46);
	cv::Scalar upper_green = cv::Scalar(99, 255, 255);
	cv::Scalar lower_yellow = cv::Scalar(14, 43, 0);
	cv::Scalar upper_yellow = cv::Scalar(34, 255, 255);

	cv::Mat mask1, mask2, maskr, maskg, masky;
	inRange(hsv, lower_red1, upper_red1, mask1);
	inRange(hsv, lower_red2, upper_red2, mask2);
	inRange(hsv, lower_green, upper_green, maskg);
	inRange(hsv, lower_yellow, upper_yellow, masky);
	add(mask1, mask2, maskr);

	unsigned int colorType[] = { 0,0,0 };
	unsigned int color = 0;
	unsigned int rows = img.rows;
	unsigned int cols = img.cols;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int colorGreen = int(maskg.at<uchar>(i, j));
			int colorRed = int(maskr.at<uchar>(i, j));
			int colorYellow = int(masky.at<uchar>(i, j));
			if (colorRed == 255) { // 统计红、黄、绿像素点的个数
				colorType[0]++;
			}
			if (colorYellow == 255) {
				colorType[1]++;
			}
			if (colorGreen == 255) {
				colorType[2]++;
			}
		}
	}

	if (colorType[0] < this->PixelNum && colorType[1] < this->PixelNum && colorType[2] < PixelNum) { //如果红、黄、绿像素点的个数都小于像素阈值，颜色置为未知，即color=0
		color = 1;
	}
	else {
		if (colorType[0] > colorType[1] && colorType[0] > colorType[2]) { // 比较像素点的数量，红色color=3；黄色color=2；绿色color=0；
			color = 3;
		}
		if (colorType[1] > colorType[0] && colorType[1] > colorType[2]) {
			color = 2;
		}
		if (colorType[2] > colorType[0] && colorType[2] > colorType[1]) {
			color = 0;
		}
	}
	tflpatchResult.countColor = color;
	return;
}

unsigned int TrafficLightClssify::TemplateMatch(int cols, int rows)
{
	auto hw_rate = cols/float(rows);
	if( this->modConfig.cols_rate*0.7 < hw_rate && hw_rate < this->modConfig.cols_rate*1.3 )
	{
		return THREE_ONE;
	}
	else if(this->modConfig.rows_rate*0.7 < hw_rate && hw_rate <this->modConfig.rows_rate*1.3)
	{
		return ONE_THREE;
	}
	else
	{
		return NO_MATCH;
	}
}


void TrafficLightClssify::ModClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult)
{
	cv::Mat gray;
	cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
	std::vector<std::vector<cv::Point>> mContours;
	Eigen::MatrixXd matrixImg(img.rows, img.cols);
	Eigen::Matrix2d::Index x, y;
	int color = -1;

	auto mod = this->TemplateMatch(img.cols, img.rows);
	if (mod == NO_MATCH) 
	{
		tflpatchResult.modColor  = TFL_OFF;
		return;
	}

	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::threshold(gray, gray, 127, 255, cv::THRESH_BINARY);
	cv::findContours(gray, mContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	cv::drawContours(mask, mContours, -1, cv::Scalar(255, 255, 255), cv::FILLED);
	cv::bitwise_and(gray, mask, gray);
	cv::morphologyEx(gray, gray, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
	
	cv::cv2eigen(gray, matrixImg);
	
	if (mod == THREE_ONE)
	{
		auto sums = matrixImg.rowwise().sum();
		sums.maxCoeff(&x, &y);
		auto idx = static_cast<float>(x);
		color = idx / matrixImg.rows() * 3;
	}
	else
	{
		auto sums = matrixImg.colwise().sum();
		sums.maxCoeff(&x, &y);
		auto idx = static_cast<float>(y);
		color = idx / matrixImg.cols() * 3;
	}
	tflpatchResult.modColor = static_cast<int>(TFL_COLOR_MAP[color]);
	return;
}


// 交通灯颜色检测(卷积网络推理)
void TrafficLightClssify::DNNClassify(const cv::Mat &img, TFLPatchResult &tflpatchResult)
{
	cv::Mat img_resize;
	cv::resize(img, img_resize, cv::Size(32, 32), 0, 0, cv::INTER_LINEAR);
	// 归一化

	int batchSize = 1;
	size_t inputSize = 3 * IMAGE_SIZE * IMAGE_SIZE;
	size_t outputSize = 1 * 4;
	float output[outputSize];
	float input[inputSize];
	for (int b = 0; b < 1; ++b) {
		for (int j = 0; j < 32 * 32; ++j) {
			int tmp = j * 3;
            input[b * 3 * 32 * 32 + j + 32 * 32 * 0] = ((float)img_resize.data[tmp + 2] - 123.675) / 58.395;   // R
			input[b * 3 * 32 * 32 + j + 32 * 32 * 1] = ((float)img_resize.data[tmp + 1] - 116.28) / 57.12;    // G 
			input[b * 3 * 32 * 32 + j + 32 * 32 * 2] = ((float)img_resize.data[tmp + 0] - 103.53) / 57.375;   // B
		}
	}

	// 读取模型进行推理
	Eigen::Index y;
	doInference(*context, input, output, batchSize, inputSize * sizeof(float), outputSize * sizeof(float));
	
	// 计算 Softmax 
	CLS eigen_output;
	for (int i = 0; i < outputSize; i++) {
		eigen_output(i) = output[i];
	}
	auto exp = eigen_output.array().exp();
	auto scores = exp / (exp.sum());
	
	scores.maxCoeff(&y);
	tflpatchResult.netColor = static_cast<int>(y);
	tflpatchResult.scores = scores;

	if(DEBUG)
		std::cout << "scores: " << scores << ", y:"<< y << std::endl;
}


void TrafficLightClssify::ColorFusion(TFLPatchResult &tflpatchResult)
{
	// simple demo
	tflpatchResult.fusionColor = tflpatchResult.netColor;
}


// 加载engine模型参数
nvinfer1::IExecutionContext* TrafficLightClssify::buildEngine(std::string engineName)
{
	char* trtModelStream{ nullptr };
	std::ifstream file(engineName, std::ios::binary);                 // 读操作
	std::fpos <mbstate_t> size;
	if (file.good()) {                                                // 判断file是否正常
		file.seekg(0, file.end);
		size = file.tellg();                                          // 得到所需的内存空间大小
		file.seekg(0, file.beg);
		trtModelStream = new char[size];                              // 申请内存空间
		assert(trtModelStream);
		file.read(trtModelStream, size);                              // 从文件里面读内容到内存中
		file.close();                                                 // 关闭文件
	}

	this->runtime = nvinfer1::createInferRuntime(gLogger);                  // 创建runtime对象
	assert(runtime != nullptr);                                       // 确保runtime对象创建成功
	this->engine = runtime->deserializeCudaEngine(trtModelStream, size);    // 反序列化创建engine
	assert(engine != nullptr);
	this->context = engine->createExecutionContext();                       // 创建context对象
	assert(context != nullptr);
	delete[] trtModelStream;
	return context;
}

// 加载engine模型参数,使用tensorrt进行推理
void TrafficLightClssify::doInference(nvinfer1::IExecutionContext& context, void* input, void* output, int batchSize, size_t inputSize, size_t outputSize)
{
	const nvinfer1::ICudaEngine& engine = context.getEngine();

	// 指向输入和输出设备缓冲区的指针，以传递给引擎。
	// 引擎需要的缓冲区数量是IEngine::getNbBindings()得到的值
	assert(engine.getNbBindings() == 2);
	void* buffers[2];

	// 为了绑定缓冲区，我们需要知道输入和输出张量的名称。
	// 请注意，保证索引小于IEngine::getNbBindings()的值
	const int inputIndex = engine.getBindingIndex(INPUT_NAME);
	const int outputIndex = engine.getBindingIndex(OUTPUT_NAME);

	// 在设备上创建GPU缓冲区
	CHECK(cudaMalloc(&buffers[inputIndex], batchSize * inputSize));
	CHECK(cudaMalloc(&buffers[outputIndex], batchSize * outputSize));

	// 创建流
	cudaStream_t stream;
	CHECK(cudaStreamCreate(&stream));

	// 这里把input的数据copy到buffer的input中，即把数据从CPU拷贝到GPU
	CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * inputSize, cudaMemcpyHostToDevice, stream));
	
	// 推理
	context.enqueueV2(buffers, stream, nullptr);

	// 把推理的结果从buffer copy到output中，即把数据从GPU copy到CPU，这步耗时较多
	CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * outputSize, cudaMemcpyDeviceToHost, stream));
	// 做同步
	cudaStreamSynchronize(stream);

	// 释放流和GPU缓冲区
	cudaStreamDestroy(stream);
	CHECK(cudaFree(buffers[inputIndex]));
	CHECK(cudaFree(buffers[outputIndex]));
}