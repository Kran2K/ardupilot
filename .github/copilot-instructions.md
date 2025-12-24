# ArduPilot AI Agent Instructions

## Project Overview

ArduPilot is a mature autopilot software for multi-rotor, fixed-wing, rover, submarine, and tracker vehicles. The codebase spans **five vehicle types** (ArduCopter, ArduPlane, Rover, ArduSub, AntennaTracker) with shared core libraries in `libraries/`.

**Key Understanding**: Each vehicle is built from a common library foundation (AP_* libraries for sensors, attitude, control, communication) with vehicle-specific code adding specialized modes, parameters, and behaviors.

## Architecture Patterns

### Core Vehicle Architecture

- **Libraries (`libraries/`)**: 90+ reusable AP_* modules (AP_InertialSensor, AP_AHRS, AP_Motors, etc.)
- **Vehicle Classes**: `Copter`, `Plane`, `Rover`, etc. contain main loop, scheduling, parameters
- **Mode System**: Each flight mode inherits from base `Mode` class (see [mode.h](ArduCopter/mode.h#L1))
- **Parameters**: Centralized in `Parameters.h` with format version tracking for EEPROM compatibility

### Key Integration Points

1. **Scheduler**: Tasks run at specific rates (50Hz, 10Hz, etc.) - look for `SCHED_TASK` macros in vehicle .cpp files
2. **MAVLink Communication**: `GCS_Mavlink.cpp` handles all ground station protocol
3. **Sensor Pipeline**: Inertial sensors → AHRS (attitude estimation) → Controllers → Motors
4. **Control Flow**: Input (RC/Guided) → Mode-specific logic → AttitudeControl → MotorControl

## Build System (WAF)

**Never use `waf` with `sudo`** - causes permission issues.

```bash
./waf configure --board=sitl           # SITL (simulator) - always use for testing
./waf configure --board=CubeBlack      # Hardware board
./waf copter                           # Build ArduCopter
./waf plane                            # Build ArduPlane
./waf --targets tests/test_math        # Build specific unit test
./waf --program-group tests            # Build all unit tests
./waf list_boards                      # Show supported boards
./waf list                             # Show all build targets
```

**Critical**: Run `waf configure` only when switching boards or build options. Build artifacts are in `build/<board>/bin/`.

## Testing

### SITL (Software-in-the-Loop) Simulation
- Located in `Tools/autotest/` with vehicle-specific test suites (arducopter.py, arduplane.py, etc.)
- Simulates vehicle dynamics and sensor inputs
- Used for pre-flight validation before hardware testing

### Unit Tests
- Found in `tests/` directory
- Use Google Test framework
- Build with: `./waf --targets tests/test_name`
- Run: `./waf check` builds all and runs relevant tests

### AutoTest Framework
- Comprehensive integration tests using SITL
- Python-based test definitions
- Tests modes, failsafes, navigation, sensor fusion
- Reference: [Tools/autotest/autotest.py](Tools/autotest/autotest.py)

## Common Patterns to Know

### Parameters (Persistent Configuration)
- Defined in `Parameters.h` enum with unique IDs
- Stored in vehicle EEPROM, loaded via AP_Param library
- Each parameter has `k_param_` prefix enum entry
- When adding: increment version, place in logical group, use next available ID

### Mode Implementation
- All modes inherit from `Mode` base class
- Implement: `init()` (setup), `run()` (main loop), `exit()` (cleanup)
- Modes access vehicle state via friend class declarations (see [Copter.h](ArduCopter/Copter.h#L200))
- Example: [ArduCopter/mode_guided.cpp](ArduCopter/mode_guided.cpp) for guided mode specifics

### Task Scheduling
- Each task has: name, rate (Hz), max execution time (μs), priority
- Defined in vehicle .cpp file using `SCHED_TASK` macro
- Critical: Respect timing - attitude control runs at 50Hz, outer loops at 10Hz
- Check [ArduCopter/Copter.cpp](ArduCopter/Copter.cpp#L78) for task definitions

### Library Dependencies
- All vehicle code includes base `#include "Copter.h"` (or equivalent)
- Libraries use inclusion guards with `.h` extension
- No circular dependencies - libraries know only about AP_Common, not vehicles
- MAVLink generated from [libraries/GCS_MAVLink/](libraries/GCS_MAVLink/)

## Critical Conventions

1. **Naming**: `class Mode*` (ModeGuided), `k_param_*` (enums), `AP_*` (libraries)
2. **Memory**: Embedded system - avoid dynamic allocation in loops, use fixed-size arrays
3. **Fixed Point Math**: Time-critical code uses integer timestamps, not float
4. **Logging**: Use AP_Logger for flight data - `logger.Write_*()` methods
5. **Error Handling**: Check return codes; avoid exceptions (embedded constraint)

## Debugging Workflow

1. **Compilation**: `./waf --verbose copter` shows detailed compiler output
2. **SITL Debugging**: Run simulator with `Tools/autotest/sim_vehicle.py` and GCS like MAVProxy or Mission Planner
3. **Log Analysis**: Extract flight logs and analyze with tools in `Tools/` (param_metadata, etc.)
4. **Parameter Testing**: Modify via GCS, parameters persist across boots

## When Modifying Code

- **New sensor driver**: Add to appropriate AP_* library (AP_Compass, AP_GPS, etc.)
- **New flight mode**: Inherit from Mode, add to [mode.h](ArduCopter/mode.h) factory, register in [GCS_Copter.cpp](ArduCopter/GCS_Copter.cpp)
- **Shared feature** (multiple vehicles): Put in libraries/, reference with `#include <AP_Feature/AP_Feature.h>`
- **Vehicle-specific**: Place in vehicle directory (ArduCopter/, ArduPlane/, etc.)

## Documentation References

- **Developer Wiki**: https://ardupilot.org/dev/
- **Copter Wiki**: https://ardupilot.org/copter/
- **Contribution Guidelines**: https://ardupilot.org/dev/docs/contributing.html
- **This README**: [README.md](README.md), [BUILD.md](BUILD.md)
