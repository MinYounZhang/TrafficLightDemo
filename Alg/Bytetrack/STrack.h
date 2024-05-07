#pragma once
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "BytekalmanFilter.h"
#include "TrafficLightInfo.h"
#include "cmc.h"
enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
	STrack(TFLPatchResult &patch, int max_cache, bool mod=true, float decay_rate=0.8);
	~STrack();

	 std::vector<float> static tlbr_to_tlwh( std::vector<float> &tlbr);
	void static multi_predict( std::vector<STrack*> &stracks, byte_kalman::ByteKalmanFilter &kalman_filter);
	void static mutil_cmc( std::vector<STrack*> &stracks, Eigen::Matrix<float,2,3> H); 
	void static_tlwh();
	void static_tlbr();
	 std::vector<float> tlwh_to_xyah( std::vector<float> tlwh_tmp);
	 std::vector<float> to_xyah();
	 std::vector<float> tlwh_to_xywh( std::vector<float> tlwh_tmp);
	 std::vector<float> to_xywh();
	void mark_lost();
	void mark_removed();
	int next_id();
	int end_frame();
	
	void activate(byte_kalman::ByteKalmanFilter &kalman_filter, int frame_id);
	void re_activate(STrack &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);
	void _check_append(STrack &new_track, int time_since_update);
	
private:
	byte_kalman::ByteKalmanFilter kalman_filter;
public:
	bool is_activated;
	int track_id;
	int state;

	 std::vector<float> _tlwh;
	 std::vector<float> tlwh;
	 std::vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;

	// classify label smoothing _ Ming
	CLS AVG_ARRAY;
	float decay_rate;
	bool mod; // using DNN only or graphic method 
	int max_cache;
	int head = 0;
	int rear = 0;
	Eigen::MatrixX4f FEATURE_ARRAY; // Eigen矩阵做循环队列，头指针head，尾指针rear，大小max_cache
	std::vector<int> results_array; // 计算轨迹中，分类相同的连续帧数，若大于max_same_frames，则输出当前轨迹分类结果，否则为-1
	int max_same_frames; // results_array连续max_same_frames帧结果一致，则输出结果
	int same_results_len;
	int same_results_head;
	int track_cls_result; // 当轨迹中状态满足要求时输出
};