#include "AP_HIL.h"

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
    _state.last_update_ms = 0;
}

void AP_HIL::init() {

}

void AP_HIL::handle_hil_sensor(const mavlink_message_t &msg)
{
    if (!is_enabled()) {
        return;
    }

    mavlink_hil_sensor_t packet;
    mavlink_msg_hil_sensor_decode(&msg, &packet);

    Compass &compass = AP::compass();
    if (compass.healthy()) {
        //todo
    }

    AP_Baro &baro = AP::baro();
    if (baro.healthy()) {
        //todo
    }
}

void AP_HIL::handle_hil_state_quaternion(const mavlink_message_t &msg)
{
    if (!is_enabled()) {
        return;
    }

    mavlink_hil_state_quaternion_t packet;
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet);

    static const float MG_TO_MSS = 9.80665f * 0.001f;
    static const float CM_TO_M = 0.01f;

    WITH_SEMAPHORE(_sem);

    _state.last_update_ms = AP_HAL::millis();

    _state.quat = Quaternion(packet.attitude_quaternion[0], 
                             packet.attitude_quaternion[1], 
                             packet.attitude_quaternion[2], 
                             packet.attitude_quaternion[3]);

    _state.gyro = Vector3f(packet.rollspeed, packet.pitchspeed, packet.yawspeed);

    _state.accel = Vector3f(packet.xacc, packet.yacc, packet.zacc) * MG_TO_MSS;

    _state.loc.lat = packet.lat;
    _state.loc.lng = packet.lon;
    _state.loc.alt = packet.alt / 10; // mm -> cm
    _state.loc.relative_alt = 0;
    _state.loc.terrain_alt = 0;

    _state.vel = Vector3f(packet.vx, packet.vy, packet.vz) * CM_TO_M;

    _state.airspeed = packet.ind_airspeed * CM_TO_M;
}

bool AP_HIL::get_hil_quat(Quaternion& out_quat) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_quat = _state.quat;
    return true;
}

bool AP_HIL::get_hil_location(Location& out_loc) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_loc = _state.loc;
    return true;
}

bool AP_HIL::get_hil_vel(Vector3f& out_vel) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_vel = _state.vel;
    return true;
}

bool AP_HIL::get_hil_gyro(Vector3f& out_gyro) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_gyro = _state.gyro;
    return true;
}

bool AP_HIL::get_hil_accel(Vector3f& out_accel) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_accel = _state.accel;
    return true;
}

bool AP_HIL::get_hil_airspeed(float& out_airspeed) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_airspeed = _state.airspeed;
    return true;
}