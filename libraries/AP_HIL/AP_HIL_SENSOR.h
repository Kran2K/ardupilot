#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class AP_HIL_SENSOR {
public:
    AP_HIL_SENSOR();
    
    // HIL_SENSOR 데이터 저장
    void update_sensor_data(const Vector3f& gyro, const Vector3f& accel, const Vector3f& mag);
    
    // RAW_IMU/SCALED_IMU2용 HIL_SENSOR 자이로 데이터 가져오기
    bool get_hil_sensor_gyro_millirad(Vector3f& out_gyro_millirad) const;
    
    // RAW_IMU/SCALED_IMU2용 HIL_SENSOR 가속도 데이터 가져오기
    bool get_hil_sensor_accel(Vector3f& out_accel) const;
    
    // RAW_IMU/SCALED_IMU2용 HIL_SENSOR 자력계 데이터 가져오기
    bool get_hil_sensor_mag_milligauss(Vector3f& out_mag_mgauss) const;
    
    // 데이터 유효성 확인
    bool is_data_valid() const;
    
private:
    struct {
        Vector3f gyro;           // [rad/s] from HIL_SENSOR
        Vector3f accel;          // [m/s^2] from HIL_SENSOR
        Vector3f mag;            // [gauss] from HIL_SENSOR
        uint32_t last_update_ms; // 마지막 업데이트 시간
    } _sensor_data;
    
    static constexpr uint32_t DATA_TIMEOUT_MS = 1000; // 1초 타임아웃
};
