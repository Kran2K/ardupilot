#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

/*
  Hardware-in-the-Loop Simulation (HILS) support for airspeed and altitude data.
  Provides HILS-first data access with automatic fallback to sensor-based estimation.
*/

class AP_Airspeed_HILS {
public:
    // Unit conversion constants
    static constexpr float CM_TO_M = 0.01f;
    static constexpr uint32_t CM_TO_MM = 10UL;
    static constexpr float M_TO_CM = 100.0f;
    
    // HIL-first altitude access (HIL → AHRS fallback)
    static bool get_hil_altitude_data(Location &loc);
    static float get_vfr_hud_alt_with_hil(bool use_dev_option, float dev_alt);
    static int32_t get_global_position_int_alt();
    static int32_t get_global_position_int_relative_alt();
    static float get_ahrs2_alt();
    
    // HIL-first airspeed access
    static bool get_hil_airspeed(float &airspeed);
    
    // HIL state check
    static bool is_hil_enabled();

private:
    static class AP_HIL* get_hil_pointer();
    static bool get_hil_location_cm(int32_t &alt_cm);
};
