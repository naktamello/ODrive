#include <math.h>
#include "odrive_main.h"
#include "utils.h"

CubicTrajectory::CubicTrajectory(Config_t& config) : config_(config) {
}

void CubicTrajectory::reset() {
    done_ = true;
    traj_head_ = 0;
    traj_tail_ = 1;
    queue_cnt_ = 0;
    queue_head_ = 0;
    queue_tail_ = 0;
}

bool CubicTrajectory::setup(uint8_t traj_id) {
    if ((queue_[queue_head_].traj_id != traj_id) || (queue_[queue_head_].t_from_start > 0))
        return false;
    CubicTrajectory::Waypoint_t p_pt;
    dequeue(&p_pt);
    running_[0] = p_pt;
    dequeue(&p_pt);
    running_[1] = p_pt;
    traj_head_ = 0;
    traj_tail_ = 1;
    done_ = false;
    return true;
}

bool CubicTrajectory::load_next() {
    CubicTrajectory::Waypoint_t p_pt;
    if (dequeue(&p_pt)) {
        running_[traj_head_] = p_pt;
        traj_head_ = (traj_head_ + 1) % 2;
        traj_tail_ = traj_head_ == 0 ? 1 : 0;
        return true;
    }
    return false;
}

bool CubicTrajectory::has_next() {
    if (queue_cnt_ < 1)
        return false;
    if (done_)
        return true;
    return running_[traj_tail_].traj_id == queue_[queue_head_].traj_id;
}

#pragma GCC push_options
#pragma GCC optimize("O0")
TrapezoidalTrajectory::Step_t CubicTrajectory::eval(float t) {
    TrapezoidalTrajectory::Step_t trajStep;
    trajStep.Ydd = 0;
    if (t > running_[traj_tail_].t_from_start) {
        if (!has_next() || !load_next()) {
            done_ = true;
            trajStep.Y = running_[traj_tail_].pos;
            return trajStep;
        }
    }
    float ti = running_[traj_head_].t_from_start;
    float tf = running_[traj_tail_].t_from_start;
    float tf_t = tf - t;
    float t_ti = t - ti;
    float tf_t_pow2 = tf_t * tf_t;
    float tf_t_pow3 = tf_t_pow2 * tf_t;
    float t_ti_pow2 = t_ti * t_ti;
    float t_ti_pow3 = t_ti_pow2 * t_ti;
    float ai = running_[traj_head_].accel;
    float af = running_[traj_tail_].accel;
    float xi = running_[traj_head_].pos;
    float xf = running_[traj_tail_].pos;
    float h = tf - ti;
    trajStep.Y = tf_t_pow3 * ai / (6 * h) + t_ti_pow3 * af / (6 * h) + (xi / h - ai * h / 6) * (tf - t) + (xf / h - af * h / 6) * (t - ti);
    trajStep.Yd = -tf_t_pow2 * ai / (2 * h) + t_ti_pow2 * af / (2 * h) + (xf - xi) / h - (af - ai) * h / 6;
    return trajStep;
}
#pragma GCC pop_options

bool CubicTrajectory::point_valid(CubicTrajectory::Waypoint_t pt) {
    if (queue_cnt_) {
        uint8_t last_item = queue_tail_ - 1;
        if (last_item < 0)
            last_item = queue_size_ - 1;
        if (queue_[last_item].traj_id != pt.traj_id)
            return true;
        return pt.t_from_start > queue_[last_item].t_from_start;
    }
    return true;
}

bool CubicTrajectory::enqueue(CubicTrajectory::Waypoint_t pt) {
    if (queue_cnt_ == queue_size_)
        return false;
    queue_[queue_tail_] = pt;
    queue_tail_ = (queue_tail_ + 1) % queue_size_;
    queue_cnt_++;

    return true;
}

bool CubicTrajectory::dequeue(CubicTrajectory::Waypoint_t* p_pt) {
    if (!queue_cnt_)
        return false;

    *p_pt = queue_[queue_head_];
    queue_head_ = (queue_head_ + 1) % queue_size_;
    queue_cnt_--;

    return true;
}

CubicTrajectory::Waypoint_t CubicTrajectory::deserialize_CAN_msg(uint32_t can_id, uint8_t* payload) {
    CubicTrajectory::Waypoint_t pt;
    uint32_t ext_id = can_id & 0x3FFFF;  // lower 18 bits
    pt.traj_id = ext_id >> 16;
    uint8_t sec = ext_id >> 12 & 0xF;
    uint32_t usec = (ext_id & 0xFFF) << 8;  // first 12 bits are in id
    usec |= payload[0];
    pt.t_from_start = (float)sec + (float)usec / 1000000;
    pt.pos = payload[1] << 24;
    pt.pos |= payload[2] << 16;
    pt.pos |= payload[3] << 8;
    pt.pos |= payload[4];
    if (pt.pos & (1 << 31))
        pt.pos |= 0xffff0000;
    pt.accel = payload[5] << 16;
    pt.accel |= payload[6] << 8;
    pt.accel |= payload[7];
    if (pt.accel & (1 << 23))
        pt.accel |= 0xffff0000;
    return pt;
}