#ifndef _CUBIC_TRAJ_H
#define _CUBIC_TRAJ_H
#include <stdlib.h>

class CubicTrajectory {
   public:
    struct Config_t {
        bool stop_at_endpoint = true;
    };

    struct Waypoint_t {
        int traj_id;
        float t_from_start;
        float accel;
        float pos;
    };

    explicit CubicTrajectory(Config_t& config);

    bool enqueue(int traj_id, float t_from_start, float accel, float pos);
    bool dequeue(CubicTrajectory::Waypoint_t& pt);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_object("config",
                make_protocol_property("stop_at_endpoint", &config_.stop_at_endpoint)
            )
            //  make_protocol_function("enqueue", *this, &CubicTrajectory::enqueue, "traj_id", "t_from_start", "accel", "pos"),
            //  make_protocol_function("dequeue", *this, &CubicTrajectory::dequeue)
        );
    }

    Axis* axis_ = nullptr;  // set by Axis constructor
    Config_t& config_;
    Waypoint_t queue_[20];
    int queue_cnt_ = 0;
    int queue_size_ = 20;
    int queue_head_ = 0;
    int queue_tail_ = 0;
};

#endif