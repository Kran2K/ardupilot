#include "AP_HIL_SENSOR.h"
#include <AP_HAL/AP_HAL.h>

AP_HIL_SENSOR::AP_HIL_SENSOR()
{
    _sensor_data.gyro.zero();
    _sensor_data.accel.zero();
    _sensor_data.mag.zero();
    _sensor_data.last_update_ms = 0;
}

void AP_HIL_SENSOR::update_sensor_data(const Vector3f& gyro, const Vector3f& accel, const Vector3f& mag)
{
    _sensor_data.gyro = gyro;
    _sensor_data.accel = accel;
    _sensor_data.mag = mag;
    _sensor_data.last_update_ms = AP_HAL::millis();
}

bool AP_HIL_SENSOR::get_hil_sensor_gyro_millirad(Vector3f& out_gyro_millirad) const
{
    if (!is_data_valid()) {
        return false;
    }
    
    // tandem-sils: 원본 rad/s 값 그대로 반환 (변환 없음)
    out_gyro_millirad = _sensor_data.gyro;
    
    return true;
}

bool AP_HIL_SENSOR::get_hil_sensor_accel(Vector3f& out_accel) const
{
    if (!is_data_valid()) {
        return false;
    }
    
    // tandem-sils: 원본 m/s^2 값 그대로 반환 (변환 없음)
    out_accel = _sensor_data.accel;
    
    return true;
}

bool AP_HIL_SENSOR::get_hil_sensor_mag_milligauss(Vector3f& out_mag_mgauss) const
{
    if (!is_data_valid()) {
        return false;
    }
    
    // tandem-sils: 원본 gauss 값 그대로 반환 (변환 없음)
    out_mag_mgauss = _sensor_data.mag;
    
    return true;
}

bool AP_HIL_SENSOR::is_data_valid() const
{
    if (_sensor_data.last_update_ms == 0) {
        return false;
    }
    
    uint32_t now = AP_HAL::millis();
    uint32_t age_ms = now - _sensor_data.last_update_ms;
    
    return age_ms < DATA_TIMEOUT_MS;
}
