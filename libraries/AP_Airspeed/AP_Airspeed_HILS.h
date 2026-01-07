#pragma once

/*
  DEPRECATED: This class has been removed.
  Use AP::hil() directly to access HIL data.
  
  Example:
    auto *hil = AP::hil();
    if (hil && hil->is_enabled()) {
        float airspeed;
        hil->get_hil_nav_airspeed(airspeed);
    }
*/
