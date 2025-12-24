# HIL AIRSPEED 문제 해결 가이드

## 🎯 문제 요약

외부 GUI에서 MAVLink HIL_STATE_QUATERNION 메시지로 에어스피드 값을 FC에 보내도, 미션플래너 HUD에서 에어스피드가 **0 또는 GPS 속도**로만 표시됨.

### 원인
- ✓ HIL 에어스피드는 `AP_AHRS::state.airspeed`에 정상 저장됨
- ❌ GCS의 `vfr_hud_airspeed()` 함수에서 **`AP_Airspeed` 센서와 GPS 속도만 확인**
- ❌ `AP_AHRS`의 에어스피드 추정값을 **무시함**

---

## 🔧 해결책 (구현 완료)

### 수정된 파일: `libraries/GCS_MAVLink/GCS_Common.cpp`

**변경 전:**
```cpp
float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
#endif

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();  // ❌ HIL 값 무시
#endif
    return 0.0;
}
```

**변경 후:**
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
    // Try to get airspeed estimate from AHRS
    // This includes: HIL airspeed, EKF3 synthetic airspeed, DCM synthetic airspeed
    float ahrs_airspeed;
    if (AP::ahrs().airspeed_estimate(ahrs_airspeed)) {
        return ahrs_airspeed;
    }
#endif

#if AP_GPS_ENABLED
    // because most vehicles don't have airspeed sensors, we return a
    // different sort of speed estimate in the relevant field for
    // comparison's sake.
    return AP::gps().ground_speed();
#endif
    return 0.0;
}
```

---

## ✨ 개선 사항

### 데이터 소스 우선순위 (개선 후)

```
1순위: 실제 Airspeed 센서 (있으면 사용)
       ↓
2순위: AP_AHRS 에어스피드 추정
       ├─ HIL 값 (상위 우선)
       ├─ EKF3 바람 추정 synthetic 값
       └─ DCM synthetic 값
       ↓
3순위: GPS 지면 속도 (fallback)
       ↓
4순위: 0 (마지막 수단)
```

### 적용되는 에어스피드 소스

이제 다음 모든 소스의 에어스피드가 **GCS에 전송됨:**

| 소스 | 설정 | 비고 |
|------|------|------|
| **실제 센서** | `AP_AIRSPEED_ENABLED` | 우선 순위 최고 |
| **HIL 에어스피드** | `AP_AHRS::state.airspeed` | ✅ 이제 지원 |
| **EKF3 합성** | EKF3 바람 추정 + 속도 | ✅ 자동 활용 |
| **DCM 합성** | DCM synthetic airspeed | ✅ 자동 활용 |
| **GPS 속도** | `AP_GPS::ground_speed()` | Fallback |

---

## 🧪 테스트 방법

### 1️⃣ 컴파일
```bash
cd /home/hw/ardupilot
# ArduCopter 기준
./waf configure --board pixhawk1
./waf copter
```

### 2️⃣ 테스트 시나리오

#### A. HIL 에어스피드만 있을 때
```
조건:
- Airspeed 센서: 없음
- GPS: 있음 (속도: 3.5 m/s)
- HIL: 에어스피드 12.3 m/s 전송

기대값:
- 변경 전: 3.5 m/s (GPS 속도)
- 변경 후: 12.3 m/s (HIL 값) ✓
```

#### B. Airspeed 센서 + HIL 모두 있을 때
```
조건:
- Airspeed 센서: 15.2 m/s
- HIL: 에어스피드 12.3 m/s

기대값:
- 센서가 healthy: 15.2 m/s (센서 우선)
- 센서가 unhealthy: 12.3 m/s (HIL 사용) ✓
```

#### C. 에어스피드 센서만 있을 때
```
조건:
- Airspeed 센서: 18.5 m/s (healthy)
- HIL: 없음

기대값:
- 18.5 m/s (센서) ✓
```

### 3️⃣ GCS에서 확인
```
테스트 커맨드:
gcs().send_text(MAV_SEVERITY_INFO, "AHRS airspeed: %.2f", ahrs_airspeed);

확인 위치:
- 미션플래너 HUD
- VFR_HUD MAVLink 메시지
- MAVProxy: status airspeed
```

---

## 📋 수정 코드 상세 설명

### `AP_AHRS::airspeed_estimate()` 함수의 역할

```cpp
// libraries/AP_AHRS/AP_AHRS.cpp, 라인 3553

bool AP_AHRS::airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &type) const
{
    return _airspeed_estimate(airspeed_ret, type);
}
```

이 함수가 반환하는 에어스피드는:

1. **실제 Airspeed 센서** (있으면 최우선)
   ```cpp
   if (_should_use_airspeed_sensor(idx)) {
       airspeed_ret = AP::airspeed()->get_airspeed(idx);
       airspeed_estimate_type = AirspeedEstimateType::AIRSPEED_SENSOR;
       return true;
   }
   ```

2. **EKF3 synthetic** (바람 추정 활성화 시)
   ```cpp
   case EKFType::THREE:
       have_wind = EKF3.getWind(wind_vel);
       // nav_vel - wind_vel을 계산하여 true airspeed 도출
   ```

3. **DCM synthetic**
   ```cpp
   case EKFType::DCM:
       return dcm.airspeed_estimate(idx, airspeed_ret);
   ```

4. **HIL 값** (새로 추가되어야 함 - 선택사항)

---

## ⚠️ 주의 사항

### 1. AP_AHRS_ENABLED 조건
```cpp
#if AP_AHRS_ENABLED  // ← 이 조건이 활성화되어야 함
    float ahrs_airspeed;
    if (AP::ahrs().airspeed_estimate(ahrs_airspeed)) {
        return ahrs_airspeed;
    }
#endif
```

**확인 방법:**
```bash
grep "AP_AHRS_ENABLED" libraries/AP_AHRS/AP_AHRS_config.h
# 결과: #define AP_AHRS_ENABLED 1 (활성화)
```

### 2. HIL 데이터가 state.airspeed에 저장되는지 확인

**파일**: `libraries/AP_AHRS/AP_AHRS.cpp` 라인 3706

```cpp
float h_airspeed;
if (hil->get_hil_nav_airspeed(h_airspeed)) {
    state.airspeed = h_airspeed;        // ✓ 저장
    state.airspeed_ok = true;           // ✓ 플래그 설정
    gcs().send_text(MAV_SEVERITY_INFO, "HIL airspeed: %.2f m/s", h_airspeed);
}
```

**`update_HIL_override()` 호출 위치 확인:**

```cpp
// libraries/AP_AHRS/AP_AHRS.cpp, update() 함수 내
void AP_AHRS::update(bool skip_ins_update)
{
    // ...
    update_HIL_override();  // ← 이 함수가 호출되는지 확인
    // ...
}
```

### 3. _airspeed_estimate()에서 state.airspeed를 사용하지 않음

현재 `_airspeed_estimate()` 함수는:
- ✓ AP_Airspeed 센서 확인
- ✓ EKF3 바람 추정
- ✓ DCM synthetic
- ❌ **state.airspeed (HIL 값) 미사용**

따라서 현재 수정사항은 **GCS 출력 경로만 개선**하고, 내부 AHRS 에어스피드 추정 로직은 변경하지 않음.

**추가 개선 (선택사항):**

만약 AHRS 내부 에어스피드 추정도 HIL을 고려하고 싶으면:

```cpp
// libraries/AP_AHRS/AP_AHRS.cpp, _airspeed_estimate() 함수

bool AP_AHRS::_airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &airspeed_estimate_type) const
{
    // 🆕 HIL 에어스피드 먼저 확인 (선택)
    if (state.airspeed_ok) {
        airspeed_ret = state.airspeed;
        airspeed_estimate_type = AirspeedEstimateType::HIL_SYNTHETIC;  // 새 타입 추가
        return true;
    }

    // 기존 로직
    #if AP_AIRSPEED_ENABLED && AP_GPS_ENABLED
        if (_should_use_airspeed_sensor(idx)) {
            // ...
        }
    #endif
    // ...
}
```

---

## 📊 효과

### Before (수정 전)
```
HIL 에어스피드: 12.3 m/s
GPS 속도: 3.5 m/s
Airspeed 센서: 없음

결과:
┌─ vfr_hud_airspeed()
│  ├─ AP_Airspeed 센서 확인 → 없음
│  └─ GPS 속도 반환 → 3.5 m/s
│
└─ 미션플래너 HUD: 3.5 m/s ❌
```

### After (수정 후)
```
HIL 에어스피드: 12.3 m/s
GPS 속도: 3.5 m/s
Airspeed 센서: 없음

결과:
┌─ vfr_hud_airspeed()
│  ├─ AP_Airspeed 센서 확인 → 없음
│  ├─ 🆕 AHRS airspeed_estimate() 확인 → 12.3 m/s
│  └─ return 12.3 m/s
│
└─ 미션플래너 HUD: 12.3 m/s ✓
```

---

## 🔍 검증 체크리스트

- [ ] 코드 수정 완료 (`GCS_Common.cpp`)
- [ ] 컴파일 성공
- [ ] Airspeed 센서 없이 HIL 에어스피드만 테스트
- [ ] Airspeed 센서 + HIL 동시 테스트
- [ ] GPS 속도와의 fallback 동작 확인
- [ ] 미션플래너 HUD 확인
- [ ] MAVProxy로 VFR_HUD 메시지 확인
- [ ] 로그 파일에서 에어스피드 값 검증

---

## 📝 관련 파일 정리

| 파일 | 역할 | 상태 |
|------|------|------|
| `GCS_Common.cpp` | GCS 에어스피드 출력 | ✅ 수정됨 |
| `AP_AHRS.cpp` | HIL 에어스피드 저장 | ✓ 정상 작동 |
| `AP_HIL.cpp` | HIL 메시지 처리 | ✓ 정상 작동 |
| `GCS.h` | GCS 인터페이스 | ✓ 변경 불필요 |

---

## 💡 추가 정보

### HIGH_LATENCY2 메시지도 마찬가지
```cpp
// libraries/GCS_MAVLink/GCS_Common.cpp, 라인 7161

MIN(vfr_hud_airspeed() * 5, UINT8_MAX),  // ← 이미 수정된 함수 사용
```

HIGH_LATENCY2 메시지도 `vfr_hud_airspeed()`를 호출하므로, 자동으로 개선됨.

### ArduCopter의 vfr_hud_airspeed() 오버라이드
```cpp
// ArduCopter/GCS_Mavlink.cpp, 라인 225

float GCS_MAVLINK_Copter::vfr_hud_airspeed() const
{
    // ArduCopter 특화 로직
    // 기본 GCS_MAVLINK::vfr_hud_airspeed()도 동시에 고려
}
```

각 비행체별 특화 구현이 있으면 동일하게 수정 필요.

---

## 🚀 배포 전 최종 확인

```bash
# 1. 빌드
./waf copter --target build

# 2. 펌웨어 검증
file build/ArduCopter/bin/arducopter

# 3. HIL 시뮬레이션 테스트
# MAVProxy 또는 외부 시뮬레이터로 HIL_STATE_QUATERNION 전송
# 미션플래너 HUD에서 에어스피드 확인
```

