#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_HAL/Semaphores.h>

class AP_HIL {
public:
    static AP_HIL* get_singleton() {
        return _singleton;
    }

    AP_HIL();
    
    void init();
    
    // MAVLink 핸들러
    void handle_hil_sensor(const mavlink_message_t &msg);
    void handle_hil_state_quaternion(const mavlink_message_t &msg);

    // 활성화 여부 확인
    bool is_enabled() const { return _is_enabled; }
    void set_enabled(bool enable) { _is_enabled = enable; }

    // AP_AHRS가 데이터를 빼가는 함수
    bool get_hil_quat(Quaternion& out_quat) const;
    bool get_hil_location(Location& out_loc) const;
    bool get_hil_vel(Vector3f& out_vel) const;
    bool get_hil_gyro(Vector3f& out_gyro) const;
    bool get_hil_accel(Vector3f& out_accel) const;
    bool get_hil_airspeed(float& out_airspeed) const;

private:
    static AP_HIL *_singleton;
    bool _is_enabled = true;

    mutable HAL_Semaphore _sem;
    
    struct {
        uint32_t last_update_ms;
        Quaternion quat;
        Location loc;
        Vector3f vel;
        Vector3f gyro;
        Vector3f accel;
        float airspeed;
    } _state;
};

namespace AP {
    AP_HIL *hil();
};