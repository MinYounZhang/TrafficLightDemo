#pragma once
#include<eigen3/Eigen/Dense>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/videostab/global_motion.hpp>

enum Method {
	OPTFLOW = 0,
	OPTFLOW_QUARTIC = 1
};


//Camera Motion Compensation
class CMC
{
public:
	CMC( int method, int downscale );
	~CMC();

	void apply(cv::Mat &frame, Eigen::Matrix<float,2,3> &H);
private:
	int method;
	int downscale;
    bool initFirstFrame = false;
	cv::Mat preFrame;
	std::vector<cv::Point2f> preKeyPoints;

	struct OptConfig
	{
		int maxCorners = 1000;
		double qualityLevel = 0.01;
		double minDistance = 1;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;
	} optConfig;

	void applyOptFlow(cv::Mat &frame, Eigen::Matrix<float,2,3> &H);
};