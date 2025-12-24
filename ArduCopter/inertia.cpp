#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // tandem-sils: HIL 활성화하면, 센서 기반 추정 고도 무시
    AP_HIL *hil_ptr = AP::hil();
    if (hil_ptr && hil_ptr->is_enabled()) {
        // Use HIL data directly, don't update inertial_nav with sensor data
        Location hil_loc {};
        if (hil_ptr->get_hil_nav_location(hil_loc)) {
            current_loc.lat = hil_loc.lat;
            current_loc.lng = hil_loc.lng;
            current_loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_ORIGIN);
            if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
                current_loc.set_alt_cm(hil_loc.alt, Location::AltFrame::ABOVE_HOME);
            }

            return;  // Skip all sensor-based updates
        }
    }
    
    // Normal sensor-based estimation (only when HIL is disabled)
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    inertial_nav.update(vibration_check.high_vibes);

    // pull position from ahrs
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    // Use AHRS altitude
    current_loc.set_alt_cm(loc.alt, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_cm(loc.alt, Location::AltFrame::ABOVE_HOME);
    }
}
