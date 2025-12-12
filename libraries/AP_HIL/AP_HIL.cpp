#include "AP_HIL.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>

AP_HIL *AP_HIL::_singleton;

namespace AP {
    AP_HIL *hil() {
        return AP_HIL::get_singleton();
    }
};

AP_HIL::AP_HIL() {
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_HIL must be a singleton");
    }
    _singleton = this;
}

void AP_HIL::init() {
    // any required initialisation
}

void AP_HIL::handle_hil_sensor(const mavlink_message_t &msg)
{
    if (!is_enabled()) {
        return;
    }

    mavlink_hil_sensor_t packet;
    mavlink_msg_hil_sensor_decode(&msg, &packet);

    // AP_InertialSensor &ins = AP::ins();

    Compass &compass = AP::compass();
    if (compass.healthy()) {

    }

    AP_Baro &baro = AP::baro();
    if (baro.healthy()) {

    }
}

void AP_HIL::handle_hil_state_quaternion(const mavlink_message_t &msg)
{
    if (!is_enabled()) {
        return;
    }

    mavlink_hil_state_quaternion_t packet;
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet);

    AP_AHRS &ahrs = AP::ahrs();
    
    if (!ahrs.home_is_set()) {
        return;
    }

    // Attitude quaternion
    Quaternion attitude(packet.attitude_quaternion[0], packet.attitude_quaternion[1], packet.attitude_quaternion[2], packet.attitude_quaternion[3]);

    // Location (LLA)
    Location loc;
    loc.lat = packet.lat;
    loc.lng = packet.lon;
    loc.alt = packet.alt;

    // Velocity (NED) in m/s
    Vector3f velocity(packet.vx * 1.0e-2f, packet.vy * 1.0e-2f, packet.vz * 1.0e-2f);

    uint64_t timestamp_us = AP_HAL::micros64();
    
    //ahrs.set_external_nav_data(loc, velocity, attitude, timestamp_us);
}
