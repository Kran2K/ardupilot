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
    _nav_state.last_update_ms = 0;
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

    WITH_SEMAPHORE(_sem);

    _sensor_state.last_update_ms = AP_HAL::millis();

    _sensor_state.gyro = Vector3f(packet.xgyro, packet.ygyro, packet.zgyro);

    _sensor_state.accel = Vector3f(packet.xacc, packet.yacc, packet.zacc);

    _sensor_state.mag = Vector3f(packet.xmag, packet.ymag, packet.zmag) * 1000.0f;

    _sensor_state.baro_pressure = packet.abs_pressure * 100.0f;
    _sensor_state.baro_temp = packet.temperature;

    _sensor_state.diff_pressure = packet.diff_pressure * 100.0f;
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

    _nav_state.last_update_ms = AP_HAL::millis();

    _nav_state.quat = Quaternion(packet.attitude_quaternion[0], 
                             packet.attitude_quaternion[1], 
                             packet.attitude_quaternion[2], 
                             packet.attitude_quaternion[3]);

    _nav_state.gyro = Vector3f(packet.rollspeed, packet.pitchspeed, packet.yawspeed);

    _nav_state.accel = Vector3f(packet.xacc, packet.yacc, packet.zacc) * MG_TO_MSS;

    _nav_state.loc.lat = packet.lat;
    _nav_state.loc.lng = packet.lon;
    _nav_state.loc.alt = packet.alt / 10; // mm -> cm
    _nav_state.loc.relative_alt = 0;
    _nav_state.loc.terrain_alt = 0;

    _nav_state.vel = Vector3f(packet.vx, packet.vy, packet.vz) * CM_TO_M;

    _nav_state.airspeed = packet.ind_airspeed * CM_TO_M;
}

bool AP_HIL::get_hil_nav_quat(Quaternion& out_quat) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_quat = _nav_state.quat;
    return true;
}

bool AP_HIL::get_hil_nav_location(Location& out_loc) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_loc = _nav_state.loc;
    return true;
}

bool AP_HIL::get_hil_nav_vel(Vector3f& out_vel) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_vel = _nav_state.vel;
    return true;
}

bool AP_HIL::get_hil_nav_gyro(Vector3f& out_gyro) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_gyro = _nav_state.gyro;
    return true;
}

bool AP_HIL::get_hil_nav_accel(Vector3f& out_accel) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_accel = _nav_state.accel;
    return true;
}

bool AP_HIL::get_hil_nav_airspeed(float& out_airspeed) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_airspeed = _nav_state.airspeed;
    return true;
}

// raw 자이로
bool AP_HIL::get_hil_sensor_gyro(Vector3f& out_gyro) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_gyro = _sensor_state.gyro;
    return true;
}

// raw 가속도
bool AP_HIL::get_hil_sensor_accel(Vector3f& out_accel) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_accel = _sensor_state.accel;
    return true;
}

// raw 자력계
bool AP_HIL::get_hil_sensor_mag(Vector3f& out_mag) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_mag = _sensor_state.mag;
    return true;
}

// raw baro
bool AP_HIL::get_hil_sensor_baro(float& out_pressure, float& out_temp) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_pressure = _sensor_state.baro_pressure;
    out_temp = _sensor_state.baro_temp;
    return true;
}

// raw pitot
bool AP_HIL::get_hil_sensor_diff_pressure(float& out_diff_press) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_diff_press = _sensor_state.diff_pressure;
    return true;
}