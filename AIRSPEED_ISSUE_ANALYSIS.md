# HIL AIRSPEED가 미션플래너 HUD에서 0으로 보이는 원인 분석

## 문제 상황
- 외부 GUI에서 MAVLink HIL_STATE_QUATERNION 메시지로 에어스피드 값을 보냄
- FC에서 값을 수신하여 AP_AHRS의 `state.airspeed`에 저장함
- **그러나 미션플래너 HUD에서는 에어스피드가 0으로 표시됨**

---

## 근본 원인 분석

### 1️⃣ **에어스피드 데이터 흐름의 단절**

#### 📍 문제 위치 1: GCS에서 에어스피드를 가져오는 경로

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp` (라인 3348-3365)

```cpp
float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();  // 실제 에어스피드 센서 사용
    }
#endif

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();  // ⚠️ FALLBACK: GPS 속도로 대체
#endif

    return 0.0;  // ⚠️ 최후의 보루: 0 반환
}
```

**⚠️ 문제점:**
- GCS가 에어스피드를 보낼 때 **`AP_Airspeed` 센서 객체 또는 GPS 속도만 확인**
- **`AP_AHRS`의 `state.airspeed`는 확인하지 않음**
- HIL을 통해 `state.airspeed`에 저장된 값이 **아예 사용되지 않음**

---

#### 📍 문제 위치 2: AHRS에서 에어스피드 추정

**파일**: `libraries/AP_AHRS/AP_AHRS.cpp` (라인 939-1039)

```cpp
bool AP_AHRS::_airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &airspeed_estimate_type) const
{
#if AP_AIRSPEED_ENABLED && AP_GPS_ENABLED
    if (_should_use_airspeed_sensor(idx)) {
        // ✓ 실제 센서가 있으면 사용
        airspeed_ret = AP::airspeed()->get_airspeed(idx);
        return true;
    }
#endif

    if (!get_wind_estimation_enabled()) {
        // ⚠️ 바람 추정이 꺼져있으면 0 반환
        return false;
    }

    // EKF3를 통한 synthetic 에어스피드 추정
    // (바람 벡터와 속도로부터 계산)
    ...
}
```

**⚠️ 문제점:**
- `state.airspeed`는 `update_HIL_override()`에서 설정되지만
- `_airspeed_estimate()`에서는 **`state.airspeed`를 사용하지 않음**
- 에어스피드 센서나 EKF3 바람 추정을 통해서만 값을 얻음

---

### 2️⃣ **HIL 에어스피드가 저장되지만 활용되지 않음**

**파일**: `libraries/AP_AHRS/AP_AHRS.cpp` (라인 3700-3710)

```cpp
// 대기속도
float h_airspeed;
if (hil->get_hil_nav_airspeed(h_airspeed)) {
    state.airspeed = h_airspeed;           // ✓ 저장은 됨
    state.airspeed_ok = true;
    gcs().send_text(MAV_SEVERITY_INFO, "HIL airspeed: %.2f m/s", h_airspeed);
}
```

**문제점:**
- `state.airspeed`에 저장하지만, 이 변수가 **데이터 출력 경로에 포함되지 않음**
- GCS가 `airspeed_estimate()` 함수를 호출하면 `state.airspeed` 값을 보지 않음

---

### 3️⃣ **VFR_HUD 메시지가 잘못된 소스에서 데이터를 가져옴**

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp` (라인 3385-3401)

```cpp
void GCS_MAVLINK::send_vfr_hud()
{
    CHECK_PAYLOAD_SIZE(MAVLINK_MSG_ID_VFR_HUD_LEN);

    mavlink_msg_vfr_hud_send(chan,
        vfr_hud_airspeed(),        // ⚠️ 여기서 0을 받음!
        vfr_hud_climbrate(),
        vfr_hud_throttle(),
        vfr_hud_heading(),
        global_position_current_loc.alt * 0.01f,
        vfr_hud_alt_terrain());
}
```

**데이터 흐름:**
```
vfr_hud_airspeed()
  └─> AP_Airspeed 센서 확인 (없음)
  └─> GPS 속도 반환 (또는 0)
  └─> ❌ state.airspeed 확인 안 함
```

---

## 데이터 흐름 비교

### ✓ 정상 경로 (에어스피드 센서가 있을 때)
```
Airspeed Sensor
    ↓
AP_Airspeed::get_airspeed()
    ↓
vfr_hud_airspeed()
    ↓
VFR_HUD MAVLink 메시지
    ↓
미션플래너 HUD 표시
```

### ❌ 현재 경로 (HIL 에어스피드)
```
HIL MAVLink 메시지 (AIRSPEED 값)
    ↓
AP_HIL::handle_hil_state_quaternion()
    ↓
AP_AHRS::state.airspeed 저장
    ↓
❌ vfr_hud_airspeed()에서 확인 안 함
    ↓
VFR_HUD = 0
    ↓
미션플래너 HUD = 0
```

---

## 원인 요약

| 단계 | 상태 | 문제 |
|------|------|------|
| **입력** | ✓ HIL 에어스피드 수신 | - |
| **저장** | ✓ state.airspeed에 저장 | - |
| **추정** | ✓ airspeed_estimate() 호출 | ❌ state.airspeed 사용 안 함 |
| **GCS 전송** | ❌ vfr_hud_airspeed() = 0 | ❌ AP_Airspeed만 확인 |
| **표시** | ❌ 미션플래너 HUD = 0 | 최종 원인 |

---

## 핵심 원인

**GCS가 에어스피드 값을 전송할 때 다음 우선순위로만 확인:**

1. ✓ `AP_Airspeed` 센서 객체의 값
2. ✓ GPS 지면 속도
3. ✗ **`AP_AHRS::state.airspeed` (HIL 값) ← 여기가 빠짐!**

따라서 **HIL을 통해 보낸 에어스피드가 무시되고** 에어스피드 센서나 GPS 속도로만 표시됨.

---

## 해결 방안

### 방안 1: GCS의 vfr_hud_airspeed()에 HIL 지원 추가 ⭐ **추천**

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp`

```cpp
float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
#endif

    // 🆕 HIL 에어스피드 확인 (새로 추가)
    AP_AHRS &ahrs = AP::ahrs();
    float hil_airspeed;
    if (ahrs.airspeed_estimate(hil_airspeed)) {
        return hil_airspeed;
    }

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();
#endif

    return 0.0;
}
```

**장점:**
- AHRS의 에어스피드 추정 함수를 사용하므로 일관성 있음
- 센서, EKF, HIL 모든 소스를 자동으로 활용

### 방안 2: AHRS의 _airspeed_estimate()에 state.airspeed 추가

**파일**: `libraries/AP_AHRS/AP_AHRS.cpp`

```cpp
bool AP_AHRS::_airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &airspeed_estimate_type) const
{
    // 🆕 HIL 에어스피드 먼저 확인
    if (state.airspeed_ok) {
        airspeed_ret = state.airspeed;
        airspeed_estimate_type = AirspeedEstimateType::HIL_SYNTHETIC;
        return true;
    }

    // ... 기존 코드
}
```

**장점:**
- 우선순위 명확 (HIL > 센서 > EKF 추정)
- AHRS 내부에서 완결

### 방안 3: HIGH_LATENCY2에서도 HIL 에어스피드 사용

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp` (라인 7161)

```cpp
// 현재 (잘못됨):
MIN(vfr_hud_airspeed() * 5, UINT8_MAX),  // 이미 0이 나옴

// 개선:
float airspeed_val;
if (AP::ahrs().airspeed_estimate(airspeed_val)) {
    MIN(airspeed_val * 5, UINT8_MAX),
} else {
    0
}
```

---

## 추가 확인 사항

### ❓ state.airspeed가 업데이트되는지 확인

**디버그 명령어:**
```cpp
// ArduCopter의 loop에 추가
if (hil_enabled) {
    float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AHRS airspeed: %.2f", airspeed);
    }
}
```

### ❓ update_HIL_override()가 호출되는지 확인

**`AP_AHRS::update()` 함수에서 호출 여부 확인 필요**

---

## 결론

**HIL 에어스피드가 0으로 보이는 이유:**

1. ✓ HIL 메시지에서 에어스피드를 수신하고 `state.airspeed`에 저장
2. ✓ `update_HIL_override()`에서 값을 설정
3. ❌ **GCS의 `vfr_hud_airspeed()`에서 `state.airspeed`를 확인하지 않음**
4. ❌ **에어스피드 센서와 GPS 속도만 확인하므로 HIL 값이 무시됨**
5. ❌ **결과: 미션플래너 HUD에서 0 표시**

**해결책:** 
- `GCS_MAVLINK::vfr_hud_airspeed()` 또는 `AP_AHRS::_airspeed_estimate()`에 HIL 에어스피드 소스 추가
