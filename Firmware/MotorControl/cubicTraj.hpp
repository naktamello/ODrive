#ifndef _CUBIC_TRAJ_H
#define _CUBIC_TRAJ_H

class CubicTrajectory {
   public:
    struct Config_t {
        bool stop_at_endpoint = true;
    };

    explicit CubicTrajectory(Config_t& config);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_object("config",
                make_protocol_property("stop_at_endpoint", &config_.stop_at_endpoint)
            )
        );
    }

    Axis* axis_ = nullptr;  // set by Axis constructor
    Config_t& config_;
};

#endif