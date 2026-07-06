# Hexapod Final

STM32F401RE 기반 6족 로봇 제어 프로젝트이다. EBIMU에서 roll/pitch/yaw를 UART로 받아 현재 자세를 계산하고, AX-12A Dynamixel 18개를 이용해 홈 자세, 시작 인사 동작, 자세 밸런싱을 수행한다.

현재 전시회용 기본 흐름은 다음과 같다.

```text
전원 ON
-> STM32 HAL 초기화
-> EBIMU UART 수신 시작
-> Dynamixel 초기화
-> 전체 Torque ON
-> Home position 이동
-> 시작 인사 동작
-> Home position 복귀
-> IMU 캘리브레이션
-> Balance loop 반복
```

## 하드웨어 구성

- MCU: STM32F401RE
- IMU: EBIMU
- 모터: Dynamixel AX-12A, ID 0~17
- Dynamixel 통신: USART6, 115200bps
- EBIMU 통신: USART1
- Dynamixel 방향 제어: 74HC126 + `DXL_DIR` GPIO

## 주요 파일 구조

```text
Core/Inc
  app.h          전체 앱 진입 함수 선언
  balance.h      자세 보정 함수/변수 선언
  dynamixel.h    AX-12A 통신 함수/변수 선언
  ebimu.h        EBIMU 수신/파싱 함수/변수 선언
  kinematics.h   IK, 좌표, 관절 계산 함수 선언

Core/Src
  main.c         STM32 기본 초기화 후 APP_Init(), APP_Loop() 호출
  app.c          전체 동작 순서 관리
  balance.c      IMU 기준값, roll/pitch 오차, 보정 명령 계산
  dynamixel.c    AX-12A 패킷 송신, Torque, Goal Position, home capture
  ebimu.c        UART 수신 문자열 파싱, roll/pitch/yaw 갱신
  kinematics.c   다리 좌표, IK, Dynamixel goal position 변환
```

## 현재 동작 모드

현재 `Core/Src/app.c` 기준 모드 설정은 다음과 같다.

```c
#define APP_HOME_CAPTURE_MODE          0
#define APP_IMU_TEST_MODE              0
#define APP_BALANCE_TEST_MODE          0
#define APP_DXL_TEST_MODE              0
#define APP_JOINT_DIRECTION_TEST_MODE  0
#define APP_STARTUP_WAVE_MODE          1
#define APP_AIR_WALK_MODE              0
#define APP_SEND_GOAL_MODE             1
```

즉 현재는 공중 보행 테스트를 끄고, 시작 인사 후 밸런싱만 수행하는 상태이다.

## 시작 인사 동작

전원 투입 후 home position에서 약 2초 유지한 뒤, 앞다리의 tibia/femur를 이용해 인사 동작을 수행한다.

```c
#define APP_WAVE_LEG_FEMUR_ID      1
#define APP_WAVE_LEG_TIBIA_ID      2
#define APP_WAVE_LIFT_TICK         40
#define APP_WAVE_BEND_TICK         340
#define APP_WAVE_NOD_TICK         -20
#define APP_WAVE_STEP_INTERVAL_MS  600
#define APP_WAVE_START_DELAY_MS    2000
#define APP_WAVE_TOGGLE_COUNT      8
```

`APP_WAVE_TOGGLE_COUNT 8`은 눈으로 보이는 까딱까딱 기준 약 4번에 해당한다.

## Dynamixel ID 매핑

다리 번호는 로봇 앞을 1번으로 보고 시계 방향으로 매긴다.

```text
1번 다리 = 앞        = ID 0, 1, 2
2번 다리 = 앞오른쪽  = ID 3, 4, 5
3번 다리 = 뒤오른쪽  = ID 6, 7, 8
4번 다리 = 뒤        = ID 9, 10, 11
5번 다리 = 뒤왼쪽    = ID 12, 13, 14
6번 다리 = 앞왼쪽    = ID 15, 16, 17
```

각 다리 안에서는 다음 순서이다.

```text
첫 번째 ID = coxa
두 번째 ID = femur
세 번째 ID = tibia
```

예를 들어 1번 앞다리는 다음과 같다.

```text
ID 0 = coxa
ID 1 = femur
ID 2 = tibia
```

## Home Position

현재 Dynamixel home position은 `dynamixel.c`와 `kinematics.c`에 동일하게 들어가 있다.

```c
{ 509, 465, 165,
  203, 457, 154,
  200, 448, 162,
  198, 491, 139,
  190, 474, 145,
  199, 484, 128 }
```

3개씩 한 다리이다.

```text
1번 다리: 509, 465, 165
2번 다리: 203, 457, 154
3번 다리: 200, 448, 162
4번 다리: 198, 491, 139
5번 다리: 190, 474, 145
6번 다리: 199, 484, 128
```

개발 중 home position을 다시 잡으려면 `APP_HOME_CAPTURE_MODE`를 `1`로 바꾸고 디버그에서 `dxl_measured_home[18]` 값을 확인한 뒤, 이 배열들을 새 값으로 교체한다. Capture 후 실제 구동할 때는 `APP_HOME_CAPTURE_MODE`를 다시 `0`으로 내려야 한다.

## 기구학 치수

현재 `Core/Src/kinematics.c` 기준 링크 길이는 다음과 같다.

```c
#define HEXAPOD_COXA_LEN_MM   50.0f
#define HEXAPOD_FEMUR_LEN_MM  65.0f
#define HEXAPOD_TIBIA_LEN_MM  120.0f
```

Home pose 기준 좌표는 다음과 같다.

```c
#define BALANCE_SIDE_RADIUS_MM   197.0f
#define BALANCE_FRONT_RADIUS_MM  227.5f
```

```c
leg_home_foot:
{  227.5,    0.0, -67.0 }
{  113.8, -197.0, -67.0 }
{ -113.8, -197.0, -67.0 }
{ -227.5,    0.0, -67.0 }
{ -113.8,  197.0, -67.0 }
{  113.8,  197.0, -67.0 }
```

```c
leg_base_position:
{  127.5,    0.0, 0.0 }
{   63.8, -110.4, 0.0 }
{  -63.8, -110.4, 0.0 }
{ -127.5,    0.0, 0.0 }
{  -63.8,  110.4, 0.0 }
{   63.8,  110.4, 0.0 }
```

코드상 home IK 각도는 대략 다음과 같다.

```text
femur angle ~= 160.3 deg
tibia angle ~= 138.2 deg
```

coxa 각도는 다리 방향에 따라 0, -60, -120, 180, 120, 60도 근처가 된다.

## 밸런싱 제어값

현재 `Core/Src/balance.c` 기준 튜닝값은 다음과 같다.

```c
#define BALANCE_KP_ROLL       1.40f
#define BALANCE_KD_ROLL       0.0f
#define BALANCE_KP_PITCH      1.40f
#define BALANCE_KD_PITCH      0.0f
#define BALANCE_DEADBAND_DEG  0.2f
#define BALANCE_FILTER_ALPHA  0.25f
#define BALANCE_MAX_STEP_CMD  0.8f
```

현재 `Core/Src/kinematics.c`와 `Core/Src/dynamixel.c`의 안전 범위는 다음과 같다.

```c
#define JOINT_MAX_DELTA_TICK  40
#define DXL_SAFE_DELTA_TICK   40
#define DXL_BALANCE_SPEED     55
```

## IMU 방향 기준

현재 테스트 기준은 다음과 같다.

```text
오른쪽으로 기울어짐: roll +
왼쪽으로 기울어짐:   roll -
앞으로 기울어짐:     pitch -
뒤로 기울어짐:       pitch +
```

밸런싱이 이상하게 보이면 먼저 IMU가 몸체에 단단히 고정되어 있는지 확인해야 한다. IMU가 흔들리면 몸체는 가만히 있어도 코드가 계속 기울었다고 판단해서 모터가 불필요하게 보정하고 전류가 튈 수 있다.
