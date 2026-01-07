#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HIL/AP_HIL.h>

namespace AP_tandem {

// Simple inline helpers for HIL groundspeed access
// These are convenience wrappers around AP_HIL for AHRS usage

static inline bool get_hil_groundspeed_vector(Vector2f &vxy)
{
    auto *hil = AP::hil();
    if (!hil || !hil->is_enabled()) {
        return false;
    }
    
    Vector3f vel;
    if (!hil->get_hil_nav_vel(vel)) {
        return false;
    }
    
    vxy = vel.xy();
    return true;
}

static inline bool get_hil_groundspeed(float &speed)
{
    Vector2f vxy;
    if (!get_hil_groundspeed_vector(vxy)) {
        return false;
    }
    
    speed = vxy.length();
    return true;
}

}  // namespace AP_tandem