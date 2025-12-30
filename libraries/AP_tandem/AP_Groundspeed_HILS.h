#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HIL/AP_HIL.h>

namespace AP_tandem {

class GroundspeedHILS {
public:
    // tandem-sils: HILS XY축 속도로 GS 계산(벡터)
    static inline bool get_groundspeed_vector(Vector2f &vxy)
    {
        Vector3f vel;
        if (!fetch_hil_vel(vel)) {
            return false;
        }
        vxy = vel.xy();
        return true;
    }

    // tandem-sils: HILS XY축 속도로 GS 계산(스칼라)
    static inline bool get_groundspeed(float &speed)
    {
        Vector3f vel;
        if (!fetch_hil_vel(vel)) {
            return false;
        }
        speed = vel.xy().length();
        return true;
    }

private:
    static Vector3f last_valid_vel;  // Cached valid velocity
    static bool has_valid_vel;        // Flag to track if we have received valid velocity
    
    static inline bool fetch_hil_vel(Vector3f &vel)
    {
        AP_HIL *hil = AP::hil();
        if (hil == nullptr) {
            return false;
        }
        
        Vector3f hil_vel;
        if (!hil->get_hil_nav_vel(hil_vel)) {
            return false;
        }
        
        // Update cache with new velocity value
        // Use very small threshold to accept near-zero but reject uninitialized data
        const float speed_2d = hil_vel.xy().length();
        
        // Always update the cache with the latest HIL data
        last_valid_vel = hil_vel;
        
        // Mark as having valid data once we get any reasonable velocity
        if (!has_valid_vel && speed_2d > 0.001f) {
            has_valid_vel = true;
        }
        
        // If we've never received valid data, reject to force fallback
        if (!has_valid_vel) {
            return false;
        }
        
        vel = last_valid_vel;
        return true;
    }
};

}  // namespace AP_tandem