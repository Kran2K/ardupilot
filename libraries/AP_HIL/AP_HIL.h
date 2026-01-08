#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_HAL/Semaphores.h>

class AP_HIL {
public:
    // tandem-sils: HIL 데이터 타임아웃 (ms) - 이 시간 이상 업데이트 없으면 invalid
    static constexpr uint32_t HIL_DATA_TIMEOUT_MS = 5000;
    
    // Unit conversion constants
    static constexpr float CM_TO_M = 0.01f;
    static constexpr uint32_t CM_TO_MM = 10UL;
    static constexpr float M_TO_CM = 100.0f;
    
    static AP_HIL* get_singleton() {
        return _singleton;
    }

    AP_HIL();
    
    void init();
    
    // tandem-sils: MAVLink 핸들러
    void handle_hil_sensor(const mavlink_message_t &msg);
    void handle_hil_state_quaternion(const mavlink_message_t &msg);

    // tandem-sils: 활성화 여부 확인
    bool is_enabled() const { return _is_enabled; }
    void set_enabled(bool enable) { _is_enabled = enable; }

    // tandem-sils: AP_AHRS가 데이터를 빼가는 함수
    bool get_hil_nav_quat(Quaternion& out_quat) const;
    bool get_hil_nav_location(Location& out_loc) const;
    bool get_hil_nav_vel(Vector3f& out_vel) const;
    bool get_hil_nav_gyro(Vector3f& out_gyro) const;
    bool get_hil_nav_accel(Vector3f& out_accel) const;
    bool get_hil_nav_accel_raw_millig(Vector3f& out_accel_mg) const;  // tandem-sils: RAW_IMU/SCALED_IMU2용 원본 millig
    bool get_hil_nav_airspeed(float& out_airspeed) const;
    
    // tandem-sils: HIL-first altitude access (HIL → AHRS fallback)
    bool get_hil_altitude_data(Location &loc) const;
    float get_vfr_hud_alt_with_hil(bool use_dev_option, float dev_alt) const;
    int32_t get_global_position_int_alt() const;
    int32_t get_global_position_int_relative_alt() const;
    float get_ahrs2_alt() const;

    // tandem-sils: 센서가 데이터를 빼가는 함수
    bool get_hil_sensor_baro(float& out_pressure, float& out_temp) const;
    bool get_hil_sensor_baro_alt(float& out_alt) const;
    bool get_hil_sensor_diff_pressure(float& out_diff_press) const;
    
    // tandem-sils: HIL_SENSOR IMU 데이터 접근 (RAW_IMU/SCALED_IMU 오버라이드용)
    bool get_hil_sensor_gyro(Vector3f& out_gyro) const;  // rad/s
    bool get_hil_sensor_accel(Vector3f& out_accel) const;  // m/s^2
    bool get_hil_sensor_mag(Vector3f& out_mag) const;  // gauss
    
    // tandem-sils: HIL 센서 데이터 통합 접근 (센서별로 HIL 우선 사용)
    bool get_hil_sensor_data(float* abs_pressure, float* diff_pressure, float* pressure_alt, float* temperature) const;

private:
    static AP_HIL *_singleton;
    bool _is_enabled = true;
    
    mutable HAL_Semaphore _sem;
    
    // tandem-sils: Private helper for altitude functions
    bool get_hil_location_alt_mm(int32_t &alt_mm) const;
    
    struct {
        uint32_t last_update_ms;
        Quaternion quat;
        Location loc;
        Vector3f vel;
        Vector3f gyro;
        Vector3f accel;
        Vector3f accel_raw_millig;  // tandem-sils: HIL_STATE_QUATERNION 원본 millig 값
        float airspeed;
    } _nav_state;

    struct SensorState {
        uint32_t last_update_ms;
        Vector3f gyro;          // [rad/s] Raw Gyro
        Vector3f accel;         // [m/s^2] Raw Accel
        Vector3f mag;           // [mGauss] Raw Magnetometer
        float baro_pressure;    // [Pa] Absolute Pressure
        float baro_temp;        // [degC] Temperature
        float pressure_alt;     // [m] Pressure Altitude
        float diff_pressure;    // [Pa] Differential Pressure (for Airspeed Sensor)
    } _sensor_state;
};

namespace AP {
    AP_HIL *hil();
};