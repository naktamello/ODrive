#include <math.h>
#include "odrive_main.h"
#include "utils.h"

CubicTrajectory::CubicTrajectory(Config_t& config) : config_(config) {
}

bool CubicTrajectory::enqueue(int traj_id, float t_from_start, float accel, float pos) {
    if (queue_cnt_ == queue_size_)
        return false;

    queue_[queue_head_].traj_id = traj_id;
    queue_[queue_head_].t_from_start = t_from_start;
    queue_[queue_head_].accel = accel;
    queue_[queue_head_].pos = pos;
    queue_tail_ = (queue_tail_ + 1) % queue_size_;
    queue_cnt_++;

    return true;
}

 bool CubicTrajectory::dequeue(CubicTrajectory::Waypoint_t& pt) {
    if (!queue_cnt_)
        return false;

    pt = queue_[queue_head_];
    queue_head_ = (queue_head_ + 1) % queue_size_;
    queue_cnt_--;

    return true;
}