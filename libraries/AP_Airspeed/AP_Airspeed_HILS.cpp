#include "AP_Airspeed_HILS.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HIL/AP_HIL.h>
#include <AP_tandem/AP_Groundspeed_HILS.h>

extern const AP_HAL::HAL& hal;

// Static member initialization for AP_tandem::GroundspeedHILS
namespace AP_tandem {
    Vector3f GroundspeedHILS::last_valid_vel = Vector3f(0, 0, 0);
    bool GroundspeedHILS::has_valid_vel = false;
}

AP_HIL* AP_Airspeed_HILS::get_hil_pointer()
{
    return AP::hil();
}

bool AP_Airspeed_HILS::is_hil_enabled()
{
    AP_HIL *hil_ptr = get_hil_pointer();
    return (hil_ptr && hil_ptr->is_enabled());
}

// Common helper: Get HIL altitude in cm
bool AP_Airspeed_HILS::get_hil_location_cm(int32_t &alt_cm)
{
    AP_HIL *hil = get_hil_pointer();
    if (hil && hil->is_enabled()) {
        Location hil_loc {};
        if (hil->get_hil_nav_location(hil_loc)) {
            alt_cm = hil_loc.alt;
            return true;
        }
    }
    return false;
}

bool AP_Airspeed_HILS::get_hil_altitude_data(Location &loc)
{
    AP_HIL *hil_ptr = get_hil_pointer();
    if (hil_ptr && hil_ptr->is_enabled()) {
        Location hil_loc {};
        if (hil_ptr->get_hil_nav_location(hil_loc)) {
            loc.lat = hil_loc.lat;
            loc.lng = hil_loc.lng;
            loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_ORIGIN);
            if (!AP::ahrs().home_is_set() || !loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
                loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_HOME);
            }
            return true;
        }
    }
    return false;
}

float AP_Airspeed_HILS::get_vfr_hud_alt_with_hil(bool use_dev_option, float dev_alt)
{
    int32_t alt_cm;
    if (get_hil_location_cm(alt_cm)) {
        return alt_cm * CM_TO_M;
    }
    return use_dev_option ? dev_alt : 0.0f;
}

int32_t AP_Airspeed_HILS::get_global_position_int_alt()
{
    int32_t alt_cm;
    if (get_hil_location_cm(alt_cm)) {
        return alt_cm * CM_TO_MM;
    }
    
    Location ahrs_loc {};
    if (AP::ahrs().get_location(ahrs_loc)) {
        return ahrs_loc.alt * CM_TO_MM;
    }
    return 0;
}

int32_t AP_Airspeed_HILS::get_global_position_int_relative_alt()
{
    int32_t alt_cm;
    if (get_hil_location_cm(alt_cm)) {
        return alt_cm * CM_TO_MM;
    }
    
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    posD *= -1000.0f;
    return posD;
}

float AP_Airspeed_HILS::get_ahrs2_alt()
{
    int32_t alt_cm;
    if (get_hil_location_cm(alt_cm)) {
        return alt_cm * CM_TO_M;
    }
    
    Location loc {};
    if (AP::ahrs().get_secondary_position(loc)) {
        return loc.alt * CM_TO_M;
    }
    return 0.0f;
}

bool AP_Airspeed_HILS::get_hil_airspeed(float &airspeed)
{
    AP_HIL *hil = get_hil_pointer();
    if (hil && hil->is_enabled()) {
        return hil->get_hil_nav_airspeed(airspeed);
    }
    return false;
}
