#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AP_HIL {
public:
    AP_HIL();

    static AP_HIL *get_singleton() {
        return _singleton;
    }

    void init();

    // handle HIL_SENSOR Mavlink message
    void handle_hil_sensor(const mavlink_message_t &msg);

    // handle HIL_STATE_QUATERNION Mavlink message
    void handle_hil_state_quaternion(const mavlink_message_t &msg);

    // check if HIL is enabled
    bool is_enabled() const { return _enabled; }

private:
    static AP_HIL *_singleton;

    AP_Int8 _enabled;
};

namespace AP {
    AP_HIL *hil();
};
