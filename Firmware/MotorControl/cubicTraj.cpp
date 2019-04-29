#include <math.h>
#include "odrive_main.h"
#include "utils.h"

CubicTrajectory::CubicTrajectory(Config_t& config) : config_(config) {
}

void CubicTrajectory::setup() {
    CubicTrajectory::Waypoint_t p_pt;
    dequeue(&p_pt);
    running_[0] = p_pt;
    dequeue(&p_pt);
    running_[1] = p_pt;
    traj_head_ = 0;
    traj_tail_ = 1;
    done_ = false;
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
#pragma GCC push_options
#pragma GCC optimize("O0")
TrapezoidalTrajectory::Step_t CubicTrajectory::eval(float t) {
    TrapezoidalTrajectory::Step_t trajStep;
    trajStep.Ydd = 0;
    if (t > running_[traj_tail_].t_from_start) {
        bool result = load_next();
        if (!result){
            done_ = true;
            trajStep.Y = running_[traj_tail_].pos;;
            // trajStep.Yd = 0;
            return trajStep;
        }
    }
    float ti = running_[traj_head_].t_from_start;
    // int ti_int = (int) (ti*1000);
    float tf = running_[traj_tail_].t_from_start;
    // int tf_int = (int) (tf*1000);
    // int t_int = (int) (t*1000);
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
    // float x1 = pow((tf - t),3)*ai/(6*h) + pow((t-ti),3)*af/(6*h)+(xi/h-ai*h/6)*(ti-t)+(xf/h-af*h/6)*(t-ti);
    // float dx1 = -pow((tf - t),2)*ai/(2*h) + pow((t-ti),2)/(2*h) + (xf-xi)/h - (af-ai)*h/6;
    trajStep.Y = tf_t_pow3*ai/(6*h) + t_ti_pow3*af/(6*h)+(xi/h-ai*h/6)*(tf-t)+(xf/h-af*h/6)*(t-ti);
    trajStep.Yd = -tf_t_pow2*ai/(2*h) + t_ti_pow2/(2*h) + (xf-xi)/h - (af-ai)*h/6;
    // trajStep.Y = 0;
    // trajStep.Yd = 0;
    return trajStep;
}
#pragma GCC pop_options

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
    if (pt.pos & (1<<31))
        pt.pos |=0xffff0000;
    pt.accel = payload[5] << 16;
    pt.accel |= payload[6] << 8;
    pt.accel |= payload[7];
    if (pt.accel & (1<<23))
        pt.accel |= 0xffff0000;
    return pt;
}