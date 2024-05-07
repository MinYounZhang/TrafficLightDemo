#include "STrack.h"

STrack::STrack(TFLPatchResult &patch, int max_cache, bool mod, float decay_rate):
max_cache(max_cache), 
mod(mod), 
decay_rate(decay_rate),
max_same_frames(1)
{
	_tlwh.resize(4);
	_tlwh = {patch.tlbr[0], patch.tlbr[1], patch.xywh[2], patch.xywh[3]};

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = patch.detConf;
	start_frame = 0;

	results_array = std::vector<int>(max_cache, -1);
	FEATURE_ARRAY = Eigen::MatrixX4f::Zero(max_cache, 4);
	results_array[this->rear] = patch.fusionColor;
	FEATURE_ARRAY.row(this->rear) = patch.scores;
	same_results_len = 1;
	same_results_head = results_array[this->rear];
	track_cls_result = -1;

	this->AVG_ARRAY = patch.scores;
}

STrack::~STrack()
{
}

void STrack::activate(byte_kalman::ByteKalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id();

	 std::vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	 std::vector<float> xyah = tlwh_to_xywh(_tlwh_tmp); // actually xywh there
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
	 std::vector<float> xyah = tlwh_to_xywh(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	auto time_since_update = frame_id - this->frame_id;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id();
	this->_check_append(new_track, time_since_update);
}

void STrack::update(STrack &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	 std::vector<float> xyah = tlwh_to_xywh(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;
	auto time_since_update = frame_id - this->frame_id;
	this->score = new_track.score;
	this->_check_append(new_track, time_since_update);
}

void STrack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}
	// mean: xywh, y-0.5h=t, x-0.5w=l
	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	// tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	std::swap(tlbr[2], tlbr[3]); // now tlbr is tlhw, h+t=b,w+l=r 
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

 std::vector<float> STrack::tlwh_to_xyah( std::vector<float> tlwh_tmp)
{
	 std::vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

 std::vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

 std::vector<float> STrack::tlwh_to_xywh( std::vector<float> tlwh_tmp)
{
	 std::vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	return tlwh_output;
}

 std::vector<float> STrack::to_xywh()
{
	return tlwh_to_xywh(tlwh);
}

 std::vector<float> STrack::tlbr_to_tlwh( std::vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}

int STrack::end_frame()
{
	return this->frame_id;
}

void STrack::multi_predict( std::vector<STrack*> &stracks, byte_kalman::ByteKalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[6] = 0;
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
	}
}


void STrack::mutil_cmc( std::vector<STrack*> &stracks, Eigen::Matrix<float,2,3> H)
{
	if (stracks.size() > 0)
	{
		Eigen::MatrixXf R = H.block(0, 0, 2, 2);
   		Eigen::VectorXf t = H.block(0, 2, 2, 1);
		Eigen::Matrix<float, 8, 8> R8x8 = Eigen::Matrix<float, 8, 8>::Identity();
		R8x8.block(0, 0, 2, 2) = R;
		
		for(auto track : stracks)
		{
			track->mean = R8x8 * track->mean.transpose();
			track->mean.head(2) += t;
			track->covariance = R8x8 * track->covariance * R8x8.transpose();
		}
	}
}


// 尝试将分类特征加入循环队列, 同时计算Strack状态
void STrack::_check_append( STrack &new_track, int time_since_update)
{
	Eigen::Index y;
	rear = (rear + 1) % max_cache;
	if (rear == head)
	{
		head = (head + 1) % max_cache;
	}
	AVG_ARRAY = decay_rate * AVG_ARRAY + (1 - decay_rate)*new_track.FEATURE_ARRAY.row(new_track.rear);
	FEATURE_ARRAY.row(rear) = new_track.FEATURE_ARRAY.row(new_track.rear);
	results_array[rear] = new_track.results_array[new_track.rear];
	// AVG_ARRAY.maxCoeff(&y);
	// results_array[rear] = y;
	track_cls_result = results_array[rear];
	if (false)
		std::cout << "AVG_ARRAY:" << AVG_ARRAY << ", track label:" << track_cls_result << std::endl;
	// if (results_array[rear] == same_results_head)
	// {
	// 	same_results_len++;
	// 	if (same_results_len >= max_same_frames)
	// 	{
	// 		track_cls_result = results_array[rear];
	// 	}
	// }
	// else
	// {
	// 	same_results_len = 1;
	// 	same_results_head = results_array[rear];
	// 	track_cls_result = -1;
	// }
}