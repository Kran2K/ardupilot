#include "AP_HIL.h"

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_AHRS/AP_AHRS.h>

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
    _nav_state.vel.zero();
    _nav_state.gyro.zero();
    _nav_state.accel.zero();
    _nav_state.airspeed = 0.0f;
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

    // tandem-sils: 원본 gauss 값 그대로 저장 (변환 없음)
    _sensor_state.mag = Vector3f(packet.xmag, packet.ymag, packet.zmag);

    _sensor_state.baro_pressure = packet.abs_pressure * 100.0f;  // mbar -> Pa
    _sensor_state.baro_temp = packet.temperature;  // degC
    _sensor_state.pressure_alt = packet.pressure_alt * 0.001f;  // mm -> m

    _sensor_state.diff_pressure = packet.diff_pressure * 100.0f;  // mbar -> Pa
    
    // tandem-sils: AP_HIL_SENSOR 모듈 업데이트 (RAW_IMU/SCALED_IMU2 오버라이드용)
    _hil_sensor.update_sensor_data(_sensor_state.gyro, _sensor_state.accel, _sensor_state.mag);
}

void AP_HIL::handle_hil_state_quaternion(const mavlink_message_t &msg)
{
    if (!is_enabled()) {
        return;
    }

    // tandem-sils: HILS vx,vy,vz, alt 수신
    mavlink_hil_state_quaternion_t packet;
    mavlink_msg_hil_state_quaternion_decode(&msg, &packet);

    static const float MG_TO_MSS = 9.80665f * 0.001f;

    WITH_SEMAPHORE(_sem);

    _nav_state.last_update_ms = AP_HAL::millis();

    _nav_state.quat = Quaternion(packet.attitude_quaternion[0], 
                             packet.attitude_quaternion[1], 
                             packet.attitude_quaternion[2], 
                             packet.attitude_quaternion[3]);

    _nav_state.gyro = Vector3f(packet.rollspeed, packet.pitchspeed, packet.yawspeed);

    _nav_state.accel = Vector3f(packet.xacc, packet.yacc, packet.zacc) * MG_TO_MSS;
    
    // tandem-sils: 원본 millig 값 저장 (RAW_IMU/SCALED_IMU2용)
    _nav_state.accel_raw_millig = Vector3f(packet.xacc, packet.yacc, packet.zacc);

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

bool AP_HIL::get_hil_nav_vel_with_age(Vector3f& out_vel, uint32_t& age_ms) const
{
    if (!is_enabled()) {
        return false;
    }
    WITH_SEMAPHORE(_sem);
    
    // tandem-sils: age만 반환, 타임아웃 체크는 호출자가 판단
    if (_nav_state.last_update_ms == 0) {
        return false;  // 한 번도 데이터를 받은 적 없음
    }
    
    uint32_t now = AP_HAL::millis();
    out_vel = _nav_state.vel;
    age_ms = now - _nav_state.last_update_ms;
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

// tandem-sils: RAW_IMU/SCALED_IMU2용 원본 millig 가속도 값 반환
bool AP_HIL::get_hil_nav_accel_raw_millig(Vector3f& out_accel_mg) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_accel_mg = _nav_state.accel_raw_millig;
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

// pressure altitude from HIL_SENSOR
bool AP_HIL::get_hil_sensor_baro_alt(float& out_alt) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    out_alt = _sensor_state.pressure_alt;
    return true;
}

// tandem-sils: HIL 센서 데이터 통합 접근 - HIL 활성화되면 모든 pressure 데이터 강제 사용
bool AP_HIL::get_hil_sensor_data(float* abs_pressure, float* diff_pressure, float* pressure_alt, float* temperature) const
{
    if (!is_enabled()) {
        return false;
    }

    WITH_SEMAPHORE(_sem);
    
    if (abs_pressure) {
        *abs_pressure = _sensor_state.baro_pressure;
    }
    if (diff_pressure) {
        *diff_pressure = _sensor_state.diff_pressure;
    }
    if (pressure_alt) {
        *pressure_alt = _sensor_state.pressure_alt;
    }
    if (temperature) {
        *temperature = _sensor_state.baro_temp;
    }
    
    return true;
}

// HIL altitude in cm (private helper)
bool AP_HIL::get_hil_location_alt_cm(int32_t &alt_cm) const
{
    if (!is_enabled()) {
        return false;
    }
    
    Location hil_loc {};
    if (!get_hil_nav_location(hil_loc)) {
        return false;
    }
    
    alt_cm = hil_loc.alt;
    return true;
}

bool AP_HIL::get_hil_altitude_data(Location &loc) const
{
    if (!is_enabled()) {
        return false;
    }
    
    Location hil_loc {};
    if (!get_hil_nav_location(hil_loc)) {
        return false;
    }
    
    loc.lat = hil_loc.lat;
    loc.lng = hil_loc.lng;
    loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_ORIGIN);
    
#if AP_AHRS_ENABLED
    if (!AP::ahrs().home_is_set() || !loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_HOME);
    }
#endif
    
    return true;
}

float AP_HIL::get_vfr_hud_alt_with_hil(bool use_dev_option, float dev_alt) const
{
    int32_t alt_cm;
    if (get_hil_location_alt_cm(alt_cm)) {
        return alt_cm * CM_TO_M;
    }
    return use_dev_option ? dev_alt : 0.0f;
}

int32_t AP_HIL::get_global_position_int_alt() const
{
    int32_t alt_cm;
    if (get_hil_location_alt_cm(alt_cm)) {
        return alt_cm * CM_TO_MM;
    }
    
#if AP_AHRS_ENABLED
    Location ahrs_loc {};
    if (AP::ahrs().get_location(ahrs_loc)) {
        return ahrs_loc.alt * CM_TO_MM;
    }
#endif
    
    return 0;
}

int32_t AP_HIL::get_global_position_int_relative_alt() const
{
    int32_t alt_cm;
    if (get_hil_location_alt_cm(alt_cm)) {
        return alt_cm * CM_TO_MM;
    }
    
#if AP_AHRS_ENABLED
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    posD *= -1000.0f;
    return posD;
#else
    return 0;
#endif
}

float AP_HIL::get_ahrs2_alt() const
{
    int32_t alt_cm;
    if (get_hil_location_alt_cm(alt_cm)) {
        return alt_cm * CM_TO_M;
    }
    
#if AP_AHRS_ENABLED
    Location loc {};
    if (AP::ahrs().get_secondary_position(loc)) {
        return loc.alt * CM_TO_M;
    }
#endif
    
    return 0.0f;
}
