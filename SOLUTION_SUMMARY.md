# 🎯 HIL AIRSPEED 문제 분석 및 해결 - 최종 요약

## 📌 한눈에 보기

| 항목 | 내용 |
|------|------|
| **문제** | 외부 GUI → HIL AIRSPEED → 미션플래너 HUD = 0 |
| **원인** | GCS가 에어스피드를 조회할 때 센서/GPS만 확인, AHRS 값 무시 |
| **해결** | `GCS_MAVLINK::vfr_hud_airspeed()`에 AHRS 확인 로직 추가 |
| **파일** | `libraries/GCS_MAVLink/GCS_Common.cpp` |
| **상태** | ✅ 구현 완료 |

---

## 🔍 문제 분석

### 데이터 흐름 추적

```
[외부 GUI]
    ↓ MAVLink HIL_STATE_QUATERNION (airspeed=12.3)
[FC의 GCS_MAVLINK::handle_hil_state_quaternion()]
    ↓
[AP_HIL::handle_hil_state_quaternion()]
    ↓ _nav_state.airspeed = 12.3 저장
[AP_AHRS::update_HIL_override()]
    ↓
[AP_AHRS::state.airspeed = 12.3 저장] ✓ 정상 저장됨
    │
    ├─→ 내부 사용: ❌ 사용되지 않음
    │
    └─→ GCS 출력: [GCS_MAVLINK::vfr_hud_airspeed()]
                    ├─ AP_Airspeed 센서? 없음
                    ├─ GPS 속도? 3.5 m/s ← 반환
                    └─ ❌ AP_AHRS::state.airspeed 무시
                        │
                        ▼
                    [VFR_HUD 메시지: 3.5 m/s 전송]
                        │
                        ▼
                    [미션플래너 HUD: 3.5 m/s 표시] ❌
```

### 핵심 문제

**`vfr_hud_airspeed()` 함수의 검사 순서:**

```cpp
if (airspeed_sensor) return sensor_value;  // 1순위
else if (gps) return gps_speed;             // 2순위 ← 여기서 반환됨
else return 0;                              // 3순위

// ❌ AP_AHRS::state.airspeed를 확인하지 않음!
```

---

## ✨ 해결 방법

### 수정된 코드

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp` (라인 3348)

```cpp
float GCS_MAVLINK::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();  // 1순위: 실제 센서
    }
#endif

#if AP_AHRS_ENABLED
    // Try to get airspeed estimate from AHRS
    // This includes: HIL airspeed, EKF3 synthetic airspeed, DCM synthetic airspeed
    float ahrs_airspeed;
    if (AP::ahrs().airspeed_estimate(ahrs_airspeed)) {
        return ahrs_airspeed;  // 2순위: AHRS 추정값 (HIL 포함!) ✅ NEW
    }
#endif

#if AP_GPS_ENABLED
    return AP::gps().ground_speed();  // 3순위: GPS 속도 (fallback)
#endif

    return 0.0;  // 4순위: 최후의 보루
}
```

### 개선된 우선순위

```
Before (문제):
├─ 1순위: Airspeed 센서
├─ 2순위: GPS 속도
└─ ❌ 3순위: 0 (HIL 값은 여기에 포함되지 않음)

After (해결):
├─ 1순위: Airspeed 센서
├─ 2순위: AP_AHRS 에어스피드 추정 ✅ NEW
│  ├─ HIL 값
│  ├─ EKF3 synthetic 값
│  └─ DCM synthetic 값
├─ 3순위: GPS 속도 (fallback)
└─ 4순위: 0
```

---

## 🧪 예상 동작

### 시나리오 1: HIL 에어스피드만 있을 때 ✅

```
입력:
- HIL 에어스피드: 12.3 m/s
- Airspeed 센서: 없음
- GPS 속도: 3.5 m/s

처리:
vfr_hud_airspeed()
  ├─ AP_Airspeed 센서? 없음
  ├─ AHRS::airspeed_estimate()? 12.3 m/s ✓ 반환
  └─ return 12.3

출력:
- VFR_HUD: 12.3 m/s ✓
- 미션플래너 HUD: 12.3 m/s ✓
```

### 시나리오 2: Airspeed 센서 + HIL 모두 있을 때 ✅

```
입력:
- Airspeed 센서: 15.2 m/s (healthy)
- HIL 에어스피드: 12.3 m/s
- GPS 속도: 3.5 m/s

처리:
vfr_hud_airspeed()
  ├─ AP_Airspeed 센서? 15.2 m/s ✓ 반환
  └─ return 15.2 (AHRS 확인 안 함)

출력:
- VFR_HUD: 15.2 m/s ✓
- 미션플래너 HUD: 15.2 m/s ✓
```

### 시나리오 3: 센서가 unhealthy할 때

```
입력:
- Airspeed 센서: 있지만 unhealthy
- HIL 에어스피드: 12.3 m/s
- GPS 속도: 3.5 m/s

처리:
vfr_hud_airspeed()
  ├─ AP_Airspeed 센서? unhealthy 스킵
  ├─ AHRS::airspeed_estimate()? 12.3 m/s ✓ 반환
  └─ return 12.3

출력:
- VFR_HUD: 12.3 m/s ✓
- 미션플래너 HUD: 12.3 m/s ✓ (자동 fallback!)
```

---

## 📊 기술 상세

### AP_AHRS::airspeed_estimate() 함수 분석

이 함수가 반환하는 값:

```cpp
bool AP_AHRS::airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &type) const
{
    return _airspeed_estimate(airspeed_ret, type);
}
```

순서대로 다음을 확인:

1. **실제 Airspeed 센서**
   - `AP_Airspeed::get_airspeed()`
   - 가장 정확한 값

2. **EKF3 synthetic airspeed**
   - 바람 추정 (wind estimation) 활용
   - `nav_velocity - wind_velocity` 계산
   - 센서가 없을 때 자동 제공

3. **DCM synthetic airspeed**
   - DCM 필터 기반
   - EKF3 비활성화 시 사용

4. **HIL 값** ✅ (새로 추가되는 경로)
   - `AP_AHRS::state.airspeed`
   - `update_HIL_override()`에서 설정

---

## ⚙️ 관련 코드 위치

### 1. HIL 데이터 수신 및 저장

**파일**: `libraries/AP_AHRS/AP_AHRS.cpp`

```cpp
// 라인 3640-3710
void AP_AHRS::update_HIL_override(void)
{
    // ...
    
    // 라인 3700-3710: 에어스피드 처리
    float h_airspeed;
    if (hil->get_hil_nav_airspeed(h_airspeed)) {
        state.airspeed = h_airspeed;        // ✓ 저장
        state.airspeed_ok = true;           // ✓ 플래그 설정
    }
}
```

### 2. GCS 출력 경로 (수정됨)

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp`

```cpp
// 라인 3348-3372 (수정 완료)
float GCS_MAVLINK::vfr_hud_airspeed() const
{
    // 1순위: AP_Airspeed 센서
    // 2순위: AP_AHRS 에어스피드 추정 ✅ NEW
    // 3순위: GPS 속도
    // 4순위: 0
}
```

### 3. VFR_HUD 메시지 전송

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp`

```cpp
// 라인 3385-3401
void GCS_MAVLINK::send_vfr_hud()
{
    mavlink_msg_vfr_hud_send(chan,
        vfr_hud_airspeed(),  // ← 이미 수정된 함수 사용
        ...
    );
}
```

### 4. HIGH_LATENCY2 메시지도 자동 개선

**파일**: `libraries/GCS_MAVLink/GCS_Common.cpp`

```cpp
// 라인 7161
MIN(vfr_hud_airspeed() * 5, UINT8_MAX),  // ← 같은 함수 사용하므로 자동 개선됨
```

---

## 🔗 데이터 흐름 (개선 후)

### VFR_HUD 메시지 생성

```
[GCS_MAVLINK::send_vfr_hud()]
    ↓
[vfr_hud_airspeed()]  ← 수정된 함수
    ├─ AP_Airspeed 센서 확인
    ├─ AP_AHRS::airspeed_estimate() 확인 ✅ NEW
    │  └─ state.airspeed 포함
    ├─ GPS 속도 확인
    └─ 0 (fallback)
    ↓ (첫 번째 성공한 값 반환)
[MAVLINK_MSG_ID_VFR_HUD]
    ↓
[GCS 채널 전송]
    ↓
[미션플래너 HUD 표시] ✅
```

---

## 🎁 부가 효과

### HIGH_LATENCY2도 자동 개선됨

HIGH_LATENCY2 메시지도 `vfr_hud_airspeed()`를 호용하므로:

```cpp
// GCS_Common.cpp, 라인 7161
MIN(vfr_hud_airspeed() * 5, UINT8_MAX),  // 에어스피드
```

따라서:
- ✅ HIGH_LATENCY 모드에서도 HIL 에어스피드 표시
- ✅ 저대역폭 링크에서도 정상 동작
- ✅ 추가 수정 불필요

---

## 🚀 배포 단계

### 1단계: 컴파일
```bash
cd /home/hw/ardupilot
./waf configure --board pixhawk1
./waf copter
```

### 2단계: 테스트
```
조건:
- Airspeed 센서: 없음
- GPS: 있음
- HIL 메시지: airspeed=12.3 m/s

확인:
- 미션플래너 HUD: 12.3 m/s ✓
- VFR_HUD 메시지: airspeed=12.3 ✓
- MAVProxy: status airspeed ✓
```

### 3단계: 배포
```
- 펌웨어 컴파일 완료
- HIL 시뮬레이션 테스트 통과
- 미션플래너에서 확인
- 배포
```

---

## 📋 수정 전후 비교

### Before
```python
외부_GUI → HIL_STATE_QUATERNION (airspeed=12.3)
              ↓
           FC 수신 & 저장 (state.airspeed=12.3)
              ↓
        vfr_hud_airspeed()
           ├─ 센서? 없음
           ├─ GPS=3.5 ← 반환
           └─ ❌ state.airspeed 무시
              ↓
        VFR_HUD = 3.5
              ↓
        미션플래너 = 3.5 ❌ (기대값: 12.3)
```

### After
```python
외부_GUI → HIL_STATE_QUATERNION (airspeed=12.3)
              ↓
           FC 수신 & 저장 (state.airspeed=12.3)
              ↓
        vfr_hud_airspeed()
           ├─ 센서? 없음
           ├─ AHRS 에어스피드? 12.3 ✓ NEW ← 반환
           └─ return 12.3
              ↓
        VFR_HUD = 12.3
              ↓
        미션플래너 = 12.3 ✅ (기대값: 12.3)
```

---

## ✅ 검증 체크리스트

- [x] 원인 분석 완료
- [x] 해결책 설계 완료
- [x] 코드 수정 완료 (`GCS_Common.cpp`)
- [ ] 컴파일 검증
- [ ] HIL 시뮬레이션 테스트
- [ ] 미션플래너 HUD 확인
- [ ] MAVProxy 검증
- [ ] 배포

---

## 💡 추가 정보

### 왜 AP_AHRS::airspeed_estimate()를 사용하는가?

1. **일관성**: AHRS 시스템 내에서 모든 에어스피드 소스를 통합 관리
2. **자동화**: 센서, EKF, DCM 등 모든 소스를 자동으로 시도
3. **유연성**: 새로운 에어스피드 소스 추가 시 GCS 수정 불필요
4. **확장성**: HIL, 센서, 추정값을 모두 같은 인터페이스로 처리

### 우선순위가 중요한 이유

```
센서 (가장 정확) > AHRS 추정 (중간) > GPS (부정확)

이 순서를 유지하면:
- 센서가 있으면: 센서 값 사용 (최고 정확도)
- 센서 없으면: AHRS 값 사용 (높은 정확도)
- 둘 다 없으면: GPS 사용 (낮은 정확도)
- 모두 없으면: 0 (에러)
```

---

## 📚 참고 문서

1. **AIRSPEED_ISSUE_ANALYSIS.md**: 상세 원인 분석
2. **AIRSPEED_FLOW_DIAGRAM.md**: 데이터 흐름 다이어그램
3. **SOLUTION_GUIDE.md**: 구현 및 테스트 가이드

---

## 🎯 최종 결론

**문제의 원본 원인:**
```
GCS의 vfr_hud_airspeed() 함수가 AP_AHRS의 에어스피드 추정값을 확인하지 않음
```

**해결책:**
```
AP_AHRS::airspeed_estimate() 호출 추가
```

**효과:**
```
✅ HIL 에어스피드 → 미션플래너 HUD 정상 표시
✅ Airspeed 센서도 계속 우선 사용
✅ GPS fallback도 유지
✅ 추가 하드웨어 수정 불필요
```

