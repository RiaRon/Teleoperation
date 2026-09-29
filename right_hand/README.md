# 오른손 Nova 2 → 오른손 LEAP Hand V1 controller

```
Right Nova 2 (#00782) → /senseglove/glove00782/rh/joint_states
  → senseglove_finger_ratio_pub.py --publish   (기본 출력 /right_hand/finger_ratios, SENSEGLOVE_INPUT.md)
  → right_hand_finger_ratio_controller.py  ◄── /right_hand/arm (std_msgs/Bool)
  → right_hand_dynamixel_bus.py (hardware 모드만) → Right LEAP Hand V1
```

왼손 시스템은 `../left_hand/`에 따로 있다(`/left_hand/*`, 왼손 장갑 #00795). 이 폴더는 오른손 장갑과 오른손 LEAP만 다룬다.
controller는 센서 종류를 모른다. `[thumb, index, middle, ring]` 0~1 ratio만 받는다.

**현재 상태: dry-run까지 완료. 실물 구동은 막혀 있다.** `config/right_hand.json`의 close 자세, port, baudrate가
TBD이고 `hardware_verified`가 모두 false라서, hardware 모드는 포트를 열기 전에 종료한다.

## 1. 기존 LEFT controller 분석 (`../left_hand/left_hand_finger_ratio_node_safe.py`)

| 항목 | 내용 |
|---|---|
| 데이터 흐름 | `/left_hand/finger_ratios` → finger_callback → clamp → `make_target_pose` → `GroupSyncWrite` (메시지마다 1회) |
| ratio 순서 | `[thumb, index, middle, ring]`, 0 = open, 1 = close |
| motor ID | `left_hand_motor_map.json`: thumb 0-3, index 4-7, middle 8-11, ring 12-15 ("smaller ID is farther from palm") |
| open/close | 같은 JSON의 `open_pose_deg`, `close_pose_deg` (연구실 왼손 실측으로 보임, 예: ID 9 open 360.53 → close 32.87) |
| 방향 처리 | 별도 sign 없음. `shortest_angle_delta(open, close)`의 부호가 방향. 360° 경계를 넘는 ID 3, 9는 `wrap_sensitive_ids` |
| raw ↔ 각도 | `deg % 360 / 360 × 4096 % 4096`, present position도 `% 4096` |
| ARM | `/left_hand/arm` true → present 읽기 → torque ON → present를 goal로 써서 유지 |
| DISARM | `/left_hand/arm` false → torque OFF (이미 DISARMED면 아무것도 안 함) |
| 시작 | 포트 열기 → torque OFF, profile velocity 쓰기 (ARM 전 write 있음) |
| shutdown | `finally` → torque OFF → 포트 닫기 |
| 한계 | joint limit 없음, ratio watchdog 없음, 운영 모드 확인 없음, baud 57600, profile velocity 8(safe 스크립트) |
| 통신 | XC330, Protocol 2.0, Torque 64, Profile Velocity 112, Goal 116, Present 132 |

연구실 왼손은 **공식 V1 ID 배치와 다르다**(공식 thumb 12-15, 연구실 thumb 0-3; baud 공식 4 000 000, 연구실 57 600).
그래서 왼손 값은 오른손에 쓰지 않았고, 공식 V1 값도 "실물 미검증"으로 둔다.

## 2. RIGHT 설정 근거 (`config/right_hand.json`)

| 항목 | 값 | 근거 | 실물 검증 |
|---|---|---|---|
| motor ID | index 0-3, middle 4-7, ring 8-11, thumb 12-15 / 손가락마다 MCP side, MCP forward, PIP, DIP | 공식 V1 조립 문서 ID 표(https://v1.leaphand.com/assembly), LEAP_Hand_API `python/main.py`(commit b0d00c8). 조립 문서상 오른손이 기본형, 왼손은 엄지 조립·손바닥만 다름 | ✗ ping 필요 |
| open pose | 16개 모두 180° | 공식 V1 home 자세: 180°(3.14159 rad)에서 horn 조립, `allegro_to_LEAPhand(zeros)` | ✗ torque OFF 읽기 필요 |
| close pose | **TBD (null)** | 공식 close 자세는 존재하지 않음 | ✗ 기록 필요 |
| direction | close − open의 부호 (close가 TBD이므로 미정) | 공식 규약: 양의 각도 = 굽힘, MCP side는 180° 중심 양방향 | ✗ |
| joint limit | 예: index MCP side 120.01~239.99°, thumb DIP 103.22~287.72° | 공식 `LEAPsim_limits` + 3.14159 rad(`angle_safety_clip`), `LEAP_Hand_Sim/assets/leap_hand/robot.urdf` 한계와 동일 | ✗ |
| baudrate | **TBD** | 공식 API 4 000 000, 연구실 왼손 57 600 → 확인 전 결정 불가 | ✗ |
| port | **TBD** | 공식 README: `/dev/serial/by-id/...` 사용 권장 | ✗ |
| profile velocity | 8 | 기존 `start_left_hand_camera_safe_system.sh` 운용값(하드웨어 속성 아님) | - |
| 운영 모드 | 3 또는 5만 허용 | 3 = position, 5 = current-based position(공식 API가 5로 설정) | 시작 시 자동 확인 |

V2/V2 Advanced 자료는 쓰지 않았다(LEAP_Hand_API 설명: "API for controlling LEAP Hand v1").

## 3. 파일

| 파일 | 역할 |
|---|---|
| `config/right_hand.json` | 오른손 설정. null = TBD, `hardware_verified`, `evidence` |
| `right_hand_controller_core.py` | ROS·SDK 없는 로직: 설정 검증, blocker, ratio 검증, 보간, 한계, 램프, ARM gate, watchdog, shutdown, `DryRunBus` |
| `right_hand_dynamixel_bus.py` | 실제 Dynamixel I/O. `dynamixel_sdk`는 hardware 모드에서만 import |
| `right_hand_finger_ratio_controller.py` | ROS 2 노드. `--mode dry-run`(기본) / `hardware` |
| `right_hand_hw_check.py` | **읽기 전용** 점검: baud 탐색, ID ping, 운영 모드, torque, 에러, present 각도, open/close 기록 |
| `right_hand_torque_off.py` | controller가 없을 때 직접 torque OFF (Disarm 스크립트의 대체 경로) |
| `LEAP-Right-Arm-Jazzy.sh`, `LEAP-Right-Disarm-Jazzy.sh` | `/right_hand/arm` ARM / DISARM. Disarm은 controller가 없으면 직접 torque OFF 시도 |
| `start_right_hand_senseglove_system.sh` | `dryrun`(기본) / `preview` / `hand` |
| `tests/test_right_hand_controller.py`, `tests/test_right_hand_integration.py` | 오른손 테스트 57개 (좌우 분리 확인 3개는 `../left_hand/`가 없으면 skip) |
| `tests/fixtures/right_hand_synthetic_test.json` | **합성 테스트 전용** 설정(`config_kind: synthetic_test`, close 값은 임의). hardware 모드가 거부함 |

## 4. LEFT / RIGHT 차이

| | LEFT (기존) | RIGHT (신규) |
|---|---|---|
| 공통 | ratio 형식·순서, open→close 선형 보간, ARM 시 present 유지, DISARM/종료 = torque OFF, 같은 레지스터 | 〃 |
| topic | `/left_hand/finger_ratios`, `/left_hand/arm` | `/right_hand/finger_ratios`, `/right_hand/arm` |
| 설정 | `left_hand_motor_map.json` | `config/right_hand.json` (`hand: right`가 아니면 로드 거부) |
| 각도 처리 | 360° wrap 허용(`% 360`, `% 4096`) | 한계가 0~360° 안에 있어 wrap 없음. present는 signed 32bit로 읽고 한계 밖이면 ARM 거부 |
| joint limit | 없음 | 모든 목표·유지값을 공식 한계로 clamp |
| NaN/Inf | 무시(9/26 수정) | 무시, finite 확인 후 clamp |
| 배열 길이 | 4 미만 거부, 5 이상 허용 | 정확히 4가 아니면 거부 |
| ARM 전 write | 시작 시 torque OFF·profile velocity 씀 | 시작 시 읽기만. 단 torque가 켜져 있으면 끔(유일한 ARM 전 write, 안전 방향) |
| startup | present 유지 후 다음 메시지에서 곧바로 목표로 | present에서 메시지당 최대 3°씩 램프 + profile velocity |
| watchdog | 없음 | ARMED 중 0.5 s 동안 유효 ratio 없으면 torque OFF |
| 시작 점검 | 없음 | 16개 ping, hardware error, 운영 모드(3/5), torque 상태 |
| hardware 조건 | 없음 | TBD·미검증 항목이 하나라도 있으면 포트를 열지 않음 |

## 5. 안전 처리

- **NaN/Inf/-Inf**: 먼저 finite 확인, 그다음 clamp. 거부 메시지는 watchdog을 갱신하지 않는다.
- **범위**: ratio는 0~1로 clamp, 목표 각도는 관절 한계로 clamp. 설정의 open/close가 한계 밖이면 설정 로드 자체가 실패.
- **ARM gate**: DISARMED 동안에는 bus에 어떤 write도 없다(목표 계산과 로그만).
- **ARM 조건**: 목표 계산 blocker 없음, 모든 present 각도가 한계 ±5° 안. 아니면 거부(잘못된 ID 배치·horn 조립 감지).
- **startup jump 방지**: ARM 때 present를 읽어 그대로 유지하고, 이후 목표로 메시지당 최대 `--max-step-deg`(3°) 이동. 실제 속도는 profile velocity로도 제한.
- **controller watchdog 정책 = torque OFF**:
  - ratio 노드는 장갑이 끊기면 스스로 open으로 램프하며 계속 발행한다. controller watchdog은 **ratio 노드 자체가 죽은 경우**만 잡는다.
  - 선택 근거: 기존 controller의 모든 안전 동작(DISARM, ARM 실패, 종료)이 torque OFF다. open 명령은 입력 없이 새로운 움직임을 만들고, 유지(hold)는 지금 알려진 문제 그 자체다.
  - 한계: torque OFF 시 손가락이 힘을 잃는다(잡고 있던 물체를 놓침). 재가동은 명시적 ARM으로만.
  - ARM 후 ratio가 한 번도 오지 않아도 0.5 s 뒤 torque OFF. ratio 노드를 먼저 켜야 한다.
- **DISARM**: 항상 torque OFF(이미 DISARMED여도). `LEAP-Right-Disarm-Jazzy.sh`는 controller가 3 s 안에 응답하지 않으면 `right_hand_torque_off.py`로 직접 끈다. 포트·baud가 TBD면 "전원 차단" 안내 후 종료.
- **shutdown**: `finally`에서 torque OFF, 실패해도 포트는 닫는다. Ctrl+C는 Jazzy에서 `KeyboardInterrupt`로 전달된다.

## 6. TBD 채우기 (다음 단계, 실물 필요)

```bash
source /opt/ros/jazzy/setup.bash && cd ~/Teleoperation/right_hand
ls /dev/serial/by-id/                                    # 오른손 U2D2 확인
python3 right_hand_hw_check.py --port /dev/serial/by-id/<오른손> --baudrates 57600,4000000
# torque OFF 상태에서 손을 펴고 / 쥔 채로:
python3 right_hand_hw_check.py --port ... --baudrates <확인된 값> --record open  --output right_open.json
python3 right_hand_hw_check.py --port ... --baudrates <확인된 값> --record close --output right_close.json
```

결과를 사람이 검토한 뒤 `config/right_hand.json`에 port, baudrate, open/close를 옮기고, 확인한 항목만
`hardware_verified`를 true로 바꾼다. open 실측이 공식 180°와 크게 다르면 horn 조립을 먼저 점검한다
(공식 troubleshooting: 90/180/270° 어긋남 → horn 재조립).

## 7. 실물 구동 직전 체크리스트

- [ ] 오른손 LEAP 전원 OFF 상태에서 배선·U2D2 연결 확인
- [ ] `ls /dev/serial/by-id/`로 오른손 U2D2 port 확인, 왼손 U2D2와 구분
- [ ] `right_hand_hw_check.py`로 baud 확인, ID 0~15 전부 응답
- [ ] ID 배치가 공식 표와 같은지 한 손가락씩 확인(손가락을 손으로 움직이며 present 변화 관찰)
- [ ] 운영 모드 3 또는 5, hardware error 0
- [ ] open 자세 기록(torque OFF, 손을 편 상태), 공식 180° 근처인지 확인
- [ ] close 자세 기록(torque OFF, 손으로 주먹 자세), 모든 값이 관절 한계 안
- [ ] 방향 확인: open→close 부호가 공식 규약(굽힘 = 양의 각도)과 맞는지, MCP side는 의도대로인지
- [ ] `config/right_hand.json` 갱신, `hardware_verified` 항목별 확인 후 true
- [ ] `python3 right_hand_finger_ratio_controller.py --mode hardware` 가 blocker 없이 시작되는지(DISARMED)
- [ ] `./LEAP-Right-Disarm-Jazzy.sh` 동작 확인(controller 켜진 상태, 꺼진 상태 둘 다)
- [ ] 비상 전원 차단 준비(손이 닿는 스위치), 두 번째 사람 대기
- [ ] SenseGlove 캘리브레이션 기록, `start_right_hand_senseglove_system.sh dryrun`으로 ratio 정상
- [ ] ratio stale 확인: SenseCom을 끄면 ratio가 open으로 램프
- [ ] controller watchdog 확인: preview 모드에서 ratio 노드를 끄면 0.5 s 후 "DISARMED by watchdog"
- [ ] 손 주변 장애물 제거, 손가락 사이 이물질 없음
- [ ] 첫 구동은 `MAX_RATIO=0.5`(스크립트 기본) 유지

## 8. 첫 실물 구동 절차 (문서만, 코드 수정 없음)

1. 오른손 LEAP 전원 ON, 모든 torque OFF 확인(`right_hand_hw_check.py`).
2. SenseCom + senseglove bringup 실행.
3. `./start_right_hand_senseglove_system.sh hand` → controller는 DISARMED로 시작, 시작 점검 로그 확인.
4. `./LEAP-Right-Disarm-Jazzy.sh` 한 번 실행해 DISARM 경로 확인.
5. 장갑 낀 손을 **편 상태**로 유지, R2 ratio 출력이 0 근처인지 확인.
6. `./LEAP-Right-Arm-Jazzy.sh` → "ARMED: holding present pose" 확인, 손이 움직이지 않아야 정상.
7. 편 손 유지 5초 → LEAP이 open 근처로 천천히 이동(램프)하는지 확인.
8. 검지만 10~20% 굽힘 → 오른손 LEAP 검지(ID 0-3)만 같은 방향으로 굽는지 → 다시 open.
9. 중지(ID 4-7) → open → 약지(ID 8-11) → open → 엄지(ID 12-15) → open, 각각 10~20%부터.
10. 방향이 반대이거나 다른 손가락이 움직이면 즉시 DISARM, 설정 재확인.
11. 작은 주먹(ratio 0.5 상한 그대로) → open.
12. `./LEAP-Right-Disarm-Jazzy.sh` → torque OFF 확인 → controller Ctrl+C.
13. 이상 징후(과열, 떨림, 소음, 빨간 LED) 시 즉시 전원 차단.
