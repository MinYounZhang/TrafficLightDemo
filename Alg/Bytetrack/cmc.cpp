#include "cmc.h"
#include <opencv2/videostab/global_motion.hpp>
#include <opencv2/videostab/motion_core.hpp>

#define DEBUG false

CMC::CMC(int method, int downscale): 
method(method), 
downscale(downscale)
{
}

CMC::~CMC()
{
}

void CMC::apply(cv::Mat &frame, Eigen::Matrix<float,2,3> &H)
{
    switch (this->method)
    {
        case Method::OPTFLOW:
            this->applyOptFlow(frame, H);
            break;
        // case Method::OPTFLOW_QUARTIC:
        //     break;
        default:
            break;
    }
    return;
}

void CMC::applyOptFlow(cv::Mat &frame, Eigen::Matrix<float,2,3> &H)
{
    int h, w;
    cv::Mat grayFrame;
    h = frame.rows;
    w = frame.cols;
    std::vector<cv::Point2f> keypoints;

    // std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
    // if (DEBUG)
    // {
    //     t0 = std::chrono::high_resolution_clock::now();
    // }

    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    if (this->downscale > 1)
    {
        cv::resize(grayFrame, grayFrame, cv::Size(w / this->downscale, h / this->downscale));
    }

    cv::goodFeaturesToTrack(
        grayFrame, keypoints,
        optConfig.maxCorners, optConfig.qualityLevel, optConfig.minDistance,
        cv::noArray(), optConfig.blockSize, optConfig.useHarrisDetector, optConfig.k
    );
    
    if (!this->initFirstFrame)
    {
        this->initFirstFrame = true;
        this->preFrame = grayFrame.clone();
        this->preKeyPoints = keypoints;
        return;
    }

    std::vector<cv::Point2f> matchedPoints, prePoints, curPoints;
    std::vector<uchar> status;
    std::vector<float> err;

    // Apply optical flow to the current frame
    cv::calcOpticalFlowPyrLK(
        this->preFrame, grayFrame,
        this->preKeyPoints, matchedPoints,
        status, err
    );

    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            prePoints.push_back(this->preKeyPoints[i]);
            curPoints.push_back(matchedPoints[i]);
        }
    }

    if (prePoints.size() > 4 && prePoints.size() == curPoints.size())
    {
        cv::Mat h = cv::estimateRigidTransform(prePoints, curPoints, true);
        cv::cv2eigen(h, H);
        
        if (this->downscale > 1.0) {
            H(0, 2) *= this->downscale;
            H(1, 2) *= this->downscale;
        }
    }
    else 
    {
        std::cout << "Warning: not enough matching points" << std::endl;
    }

    // Store the current frame and pts
    this->preFrame = grayFrame.clone();
    this->preKeyPoints = keypoints;

    // if (DEBUG)
    // {
    //     t1 = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    //     std::cout << "CMC optical: " << duration << " ms" << std::endl;
    // }
    return;
}