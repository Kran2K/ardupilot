# HIL AIRSPEED 데이터 흐름 다이어그램

## 1. 현재 시스템 (에어스피드 센서 있을 때 - 정상)

```
┌─────────────────┐
│  Airspeed       │
│  Sensor (I2C)   │
└────────┬────────┘
         │ I2C/CAN
         ▼
┌─────────────────────────────────┐
│   AP_Airspeed                   │
│   - get_airspeed()              │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  GCS_MAVLINK                    │
│  - vfr_hud_airspeed()           │
│    └─> AP_Airspeed 확인 ✓        │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  VFR_HUD MAVLink 메시지          │
│  airspeed = 15.5 m/s (정상)      │
└────────┬────────────────────────┘
         │ UDP/Serial
         ▼
┌─────────────────────────────────┐
│  미션플래너 HUD                  │
│  Airspeed = 15.5 m/s ✓          │
└─────────────────────────────────┘
```

---

## 2. 현재 시스템 (HIL 에어스피드 - 문제)

```
┌─────────────────────────────────┐
│  외부 GUI (시뮬레이션)           │
│  - MAVLink HIL_STATE_QUATERNION │
│    airspeed = 12.3 m/s          │
└────────┬────────────────────────┘
         │ UDP
         ▼
┌─────────────────────────────────┐
│  GCS_MAVLINK                    │
│  - handle_hil_state_quaternion()│
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  AP_HIL                         │
│  - handle_hil_state_quaternion()│
│  - _nav_state.airspeed = 12.3  │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  AP_AHRS                        │
│  - update_HIL_override()        │
│  - state.airspeed = 12.3 ✓      │
│  - state.airspeed_ok = true ✓   │
└────────┬────────────────────────┘
         │
         ├─────────────────────────────┐
         │                             │
         │ (저장됨)                    │ (사용되지 않음 ❌)
         ▼                             ▼
    state.airspeed           GCS_MAVLINK::vfr_hud_airspeed()
       = 12.3                 ├─> AP_Airspeed 센서 확인
       (미사용)               │   (없음)
                              ├─> GPS 속도 확인
                              │   └─> 3.5 m/s
                              ├─> ❌ state.airspeed 무시
                              └─> 3.5 반환 또는 0
                                    │
         ┌─────────────────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  VFR_HUD MAVLink 메시지          │
│  airspeed = 3.5 m/s (❌ 잘못됨) │
│  또는 = 0 m/s (❌ 최악)          │
└────────┬────────────────────────┘
         │ UDP/Serial
         ▼
┌─────────────────────────────────┐
│  미션플래너 HUD                  │
│  Airspeed = 3.5 또는 0 ❌        │
│  (기대값: 12.3)                 │
└─────────────────────────────────┘
```

---

## 3. 함수 호출 스택

### A. 정상 경로 (센서 있을 때)
```
GCS_MAVLINK::send_vfr_hud()
  ▼
GCS_MAVLINK::vfr_hud_airspeed()
  ▼
AP_Airspeed::get_singleton()
  ▼
AP_Airspeed::get_airspeed()  ✓ (값: 15.5)
  ▼
return 15.5
```

### B. HIL 경로 (현재 실패)
```
GCS_MAVLINK::send_vfr_hud()
  ▼
GCS_MAVLINK::vfr_hud_airspeed()
  ├─> AP_Airspeed 확인 ❌ (nullptr 또는 unhealthy)
  ├─> GPS 속도로 fallback
  ▼
return 3.5  (또는 0)
  
[미사용] AP_AHRS::state.airspeed = 12.3
```

### C. 개선 후 경로 (제안)
```
GCS_MAVLINK::send_vfr_hud()
  ▼
GCS_MAVLINK::vfr_hud_airspeed()
  ├─> AP_Airspeed 확인 (없음)
  ├─> 🆕 AP_AHRS::airspeed_estimate() 확인 ✓
  │   ▼
  │   AP_AHRS::_airspeed_estimate()
  │   ├─> state.airspeed 확인 ✓ (12.3)
  │   └─> return true
  │
  ▼
return 12.3  ✓ (정상)
```

---

## 4. state 구조체의 airspeed 관련 필드

```cpp
// libraries/AP_AHRS/AP_AHRS.h (또는 Backend)

struct AHRS_State {
    // ... 다른 필드들
    
    float airspeed;              // HIL에서 설정됨 ✓
    bool airspeed_ok;            // = true (HIL에서 설정)
    
    // 그런데 이 값들이...
    // GCS::vfr_hud_airspeed()에서는 절대 확인되지 않음 ❌
};
```

---

## 5. 문제 코드 위치 맵

```
ArduPilot 소스코드 구조:

libraries/
  ├── AP_AHRS/
  │   ├── AP_AHRS.h         (airspeed 멤버 선언)
  │   └── AP_AHRS.cpp
  │       ├── update_HIL_override()      [라인 3640]
  │       │   └─> state.airspeed = h_airspeed  ✓ (저장)
  │       │
  │       ├── _airspeed_estimate()       [라인 939]
  │       │   ├─> AP::airspeed() 센서만 확인
  │       │   ├─> EKF3 바람 추정만 확인
  │       │   └─> ❌ state.airspeed 무시
  │       │
  │       └── update()
  │           └─> update_HIL_override() 호출 여부?
  │
  ├── GCS_MAVLink/
  │   ├── GCS.h
  │   └── GCS_Common.cpp
  │       ├── vfr_hud_airspeed()         [라인 3348] ❌ 문제!
  │       │   ├─> AP_Airspeed 센서 확인
  │       │   ├─> GPS 속도로 fallback
  │       │   └─> ❌ AP_AHRS::state.airspeed 미확인
  │       │
  │       └── send_vfr_hud()             [라인 3385]
  │           └─> vfr_hud_airspeed() 호출
  │
  └── AP_HIL/
      ├── AP_HIL.h
      └── AP_HIL.cpp
          ├── handle_hil_state_quaternion() [라인 52]
          │   └─> _nav_state.airspeed = packet.ind_airspeed ✓
          │
          └── get_hil_nav_airspeed()      [라인 148]
              └─> return _nav_state.airspeed
```

---

## 6. 수정 전후 비교

### ❌ 현재 (잘못된) 코드
```cpp
// libraries/GCS_MAVLink/GCS_Common.cpp

float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
#endif

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();  // ❌ GPS 속도로 대체 (HIL 값 무시)
#endif

    return 0.0;
}
```

### ✅ 개선된 코드 (1번 방안)
```cpp
float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
#endif

#if AP_AHRS_ENABLED
    // 🆕 AHRS 에어스피드 추정 추가 (센서, EKF, HIL 모두 포함)
    float ahrs_airspeed;
    if (AP::ahrs().airspeed_estimate(ahrs_airspeed)) {
        return ahrs_airspeed;
    }
#endif

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();
#endif

    return 0.0;
}
```

---

## 7. 호출 우선순위

### 현재 (문제)
```
1순위: AP_Airspeed 센서
2순위: GPS 속도
❌ 누락: AP_AHRS (HIL 값)
3순위: 0
```

### 개선안
```
1순위: AP_Airspeed 센서
2순위: AP_AHRS::airspeed_estimate()
       ├─ HIL 값
       ├─ EKF3 바람 추정값
       └─ Synthetic 값
3순위: GPS 속도 (fallback)
4순위: 0
```

---

## 요약

| 구성 | 현재 상태 | 문제 |
|------|---------|------|
| **데이터 수신** | ✓ HIL 메시지 수신 | - |
| **데이터 저장** | ✓ AP_AHRS::state.airspeed 저장 | - |
| **데이터 활용** | ❌ GCS에서 미확인 | ← **여기가 문제!** |
| **출력** | ❌ VFR_HUD = 0 또는 GPS 속도 | 최종 증상 |

**해결책:** GCS의 `vfr_hud_airspeed()`에 `AP_AHRS::airspeed_estimate()` 호출 추가
