#ifndef _CUBIC_TRAJ_H
#define _CUBIC_TRAJ_H
#include <stdlib.h>

class CubicTrajectory {
   public:
    struct Config_t {
        bool stop_at_endpoint = true;
    };

    struct Waypoint_t {
        uint8_t traj_id;
        float t_from_start;
        int32_t accel;
        int32_t pos;
    };

    explicit CubicTrajectory(Config_t& config);
    void setup();
    bool load_next();
    TrapezoidalTrajectory::Step_t eval(float t);
    bool point_valid(CubicTrajectory::Waypoint_t pt);
    bool enqueue(CubicTrajectory::Waypoint_t pt);
    bool dequeue(CubicTrajectory::Waypoint_t* p_pt);
    bool has_next();
    Waypoint_t deserialize_CAN_msg(uint32_t can_id, uint8_t* payload);

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
    Waypoint_t running_[2];
    bool done_ = true;
    uint8_t traj_head_ = 0;
    uint8_t traj_tail_ = 1;
    Waypoint_t queue_[20];
    uint8_t queue_cnt_ = 0;
    uint8_t queue_size_ = 20;
    uint8_t queue_head_ = 0;
    uint8_t queue_tail_ = 0;
};

#endif