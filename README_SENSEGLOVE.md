# SenseGlove Nova 2 입력 → LEAP Hand finger ratio

카메라(MediaPipe) 대신 SenseGlove Nova 2 오른손(#00782)의 `JointState`로
기존 `/left_hand/finger_ratios` 인터페이스를 만든다. LEAP Hand controller는
NaN/Inf 입력 거부(아래 참고) 외에는 바꾸지 않았다.

## 1. 기존 카메라 기반 구조

```
카메라 ─ left_hand_camera_finger_pub_stable.py (MediaPipe, 15 Hz)
         │  PUBLISH OFF로 시작('s' 토글), max_ratio 0.7, deadzone, EMA,
         │  max_step 0.05/publish, 손 미검출 lost_timeout 후 target=open
         ▼
/left_hand/finger_ratios   std_msgs/Float32MultiArray
         │  data = [thumb, index, middle, ring], 0.0 = open, 1.0 = closed
         ├──────────────► left_hand_finger_joint_state_pub.py → /joint_states → RViz
         ▼
left_hand_finger_ratio_node_safe.py  ◄── /left_hand/arm (std_msgs/Bool)
         │  시작 시 torque OFF, DISARMED. ARM 시 현재 자세 유지 후 명령 수신
         │  손가락 ratio 1개 → 해당 손가락 모터 4개를 open↔close 자세 사이 선형 보간
         │  메시지마다 GroupSyncWrite 1회, 종료 시 torque OFF
         ▼
Dynamixel 16개 (left_hand_motor_map.json: thumb 0-3, index 4-7, middle 8-11, ring 12-15, 57600 baud)
```

- 기존 실행 스크립트(`start_left_hand_camera_*.sh`)는 ROS 2 Humble과 Python 3.10 `.venv`를
  가정한다. 연구실 PC는 Jazzy/Python 3.12이므로 `.venv`는 압축 해제에서 제외했다.
- `start_left_hand_camera_stable_system.sh`는 `backup/`에만 있는 `left_hand_finger_ratio_node.py`를
  실행한다. 현재 쓰는 것은 `start_left_hand_camera_safe_system.sh`이다.
- controller에는 입력 끊김(stale) 감시가 없다. 입력이 끊기면 마지막 목표 자세를 유지한다.
  그래서 open 복귀는 publisher 쪽 책임이다(카메라 publisher의 lost_timeout과 같은 역할).

## 2. 추가·변경 파일

| 파일 | 내용 |
|---|---|
| `senseglove_finger_ratio.py` | ROS 없는 변환 로직: 이름 기반 추출, 검증, 정규화, 합성, stale watchdog, 출력 제한 |
| `senseglove_finger_ratio_pub.py` | ROS 2 노드. 기본은 **dry-run**(publisher를 만들지 않음). `--publish`일 때만 발행 |
| `senseglove_calibration_anatomical.json` | 캘리브레이션 전 기본값(SenseGlove SDK v22 해부학 한계). 실측 샘플이 아님 |
| `senseglove_record_calibration.py` | 사용자 open/fist 자세를 각 2 s 기록해 중앙값으로 `senseglove_calibration.json` 생성 |
| `senseglove_fake_glove_pub.py` | 장갑 없이 dry-run하기 위해 기록된 자세를 재생(관절 순서 무작위, 끊김/NaN 주입 가능) |
| `tests/` | 단위 테스트 58개, 자세 fixture(`tests/fixtures/senseglove_poses.json`) |
| `left_hand_finger_ratio_node_safe.py` | **후속 수정**: NaN/Inf ratio 메시지 무시(아래 5절) |
| `start_left_hand_senseglove_system.sh`, `LEAP-Left-*-Jazzy.sh` | Jazzy용 실행/ARM/DISARM 스크립트(6절) |

## 3. SenseGlove → finger ratio 변환

입력: `/senseglove/glove00782/rh/joint_states` (`sensor_msgs/JointState`, 약 60 Hz)

1. `dict(zip(msg.name, msg.position))`로 **이름 기반** 조회: `r_{thumb,index,middle,ring}_{mcp,pip,dip}` 12개.
   pinky, `*_brake`, `palm_*`는 사용하지 않는다.
2. 관절마다 `n = clamp((value - open) / (closed - open), 0, 1)`로 0~1 정규화(방향이 반대여도 동작).
3. 손가락 ratio = MCP/PIP/DIP 정규화 값의 가중 평균(기본은 같은 가중치).
   - 카메라 ratio는 PIP 각도 하나로 만들었다. Nova 2에서 손가락 DIP는 PIP를 따라간다(샘플에서 DIP ≈ 0.9 × PIP).
     따라서 같은 가중치 평균은 대략 1/3 MCP + 2/3 PIP가 되어 카메라 입력과 성격이 비슷하다.
   - 엄지는 같은 식을 쓰지만 open/closed 값은 따로 둔다. 엄지의 `mcp/pip/dip`은 실제로 CMC/MCP/IP 굴곡이다.
4. 출력 `[thumb, index, middle, ring]`에 카메라와 같은 `max_ratio`(0.7)와 `max_step`(0.05/publish)을 적용해
   15 Hz로 발행한다.

SenseGlove 관절값 참고(`senseglove_ros` 소스 확인):
`*_mcp/pip/dip` = `HandPose::GetHandAngles()` 관절 1~3의 굴곡(Y)이고,
`*_brake`의 위치값은 첫 관절 벌림(−Z)이다. 주먹 샘플이 1.571/1.745 rad에서 멈춘 것은
SDK 해부학 한계(MCP 90°, PIP 100°, DIP 90°)에 걸려 포화된 값이다.

## 4. 캘리브레이션

- 운영용 값은 **기록한 사용자 캘리브레이션**을 쓴다. 단일 스냅샷을 하드코딩하지 않는다.
- `senseglove_calibration_anatomical.json`은 기록 전 dry-run용 기본값이다. 노드가 경고를 출력한다.

| 샘플 자세 | 해부학 기본값 출력 [T, I, M, R] |
|---|---|
| open | 0.11, 0.03, 0.02, 0.19 |
| fist | 0.55, 0.87, 1.00, 1.00 |
| index_bend | 0.11, 0.73, 0.40, 0.39 |
| pinch | 0.41, 0.53, 0.31, 0.32 |

## 5. 안전 처리

| 상황 | 처리 |
|---|---|
| 필요한 joint 누락, name/position 길이 불일치, 중복 이름 | 메시지 거부, ratio 갱신 안 함 |
| NaN/Inf, `|value| > max_abs_joint_rad`(π) | 메시지 거부 |
| 유효한 메시지가 `--stale-timeout`(0.3 s) 이상 없음(끊김 또는 거부만 연속) | target = open, `max_step`으로 서서히 open 복귀(15 Hz에서 0.7→0 약 0.9 s) |
| 첫 메시지 전 | open(0) 출력 |
| 노드 종료(Ctrl+C) | `--publish`일 때 open 명령 1회 발행 |
| dry-run(기본) | publisher 자체를 만들지 않음 |
| 발행 속도 | 15 Hz. controller가 메시지마다 57600 baud로 16개 모터 sync write(약 16 ms)를 하므로 60 Hz를 그대로 전달하면 버스가 포화된다 |
| 거부 로그 | `--log-period`마다 1회로 제한 |

**controller NaN 수정**: 기존 `clamp()`는 `max(0, min(1, x))`이다. 파이썬에서 NaN과의 비교는 모두
거짓이라 `min(1.0, nan)`이 `1.0`을 돌려주고, 결국 **NaN이 1.0(주먹)으로 바뀐다**.
`finger_callback`에서 앞의 4개 값 중 NaN/Inf가 있으면 경고를 남기고 무시하도록 했다.
(`left_hand_finger_joint_state_pub.py`의 `clamp_ratio`는 NaN을 그대로 통과시킨다. RViz 표시 전용이라 수정하지 않았다.)

## 6. 실행 (이 PC: Ubuntu 24.04 + ROS 2 Jazzy 한 대에서 모두 실행)

준비: `sudo apt install ros-jazzy-dynamixel-sdk` (controller가 쓰는 `dynamixel_sdk`, Python 3.12용).
기존 `.venv`(Python 3.10)와 Humble용 `.sh`는 이 PC에서 쓰지 않는다.

| 스크립트 | 용도 |
|---|---|
| `start_left_hand_senseglove_system.sh` | 기본 `dryrun`: ratio 로그만 출력 |
| `start_left_hand_senseglove_system.sh preview` | ratio 발행 + RViz 모델, 모터 없음 (`senseglove_calibration.json` 필요) |
| `start_left_hand_senseglove_system.sh hand` | preview + LEAP controller(DISARMED로 시작, `--profile-velocity 8`, max_ratio 0.5) |
| `LEAP-Left-Arm-Jazzy.sh` / `LEAP-Left-Disarm-Jazzy.sh` | ARM / DISARM. controller 구독을 최대 5 s 기다렸다 1회 발행, 없으면 exit 1 |

SenseCom과 senseglove bringup은 스크립트 전에 따로 실행한다. 카메라 publisher, `LEAP-Left-Fist/VSign.sh`는
같은 `/left_hand/finger_ratios`에 발행하므로 동시에 쓰지 않는다.

권장 순서:

```bash
source /opt/ros/jazzy/setup.bash && cd ~/left_hand_project
python3 -m pytest tests -q                          # 테스트
python3 senseglove_record_calibration.py            # 1. 사용자 캘리브레이션 기록
./start_left_hand_senseglove_system.sh              # 2. dryrun: 로그로 ratio 확인, SenseCom 끄면 STALE 확인
./start_left_hand_senseglove_system.sh preview      # 3. RViz 모델이 손을 따라오는지 확인
./start_left_hand_senseglove_system.sh hand         # 4. (실물 승인 후) 손을 편 상태에서 ./LEAP-Left-Arm-Jazzy.sh
```

장갑 없이 확인할 때는 `python3 senseglove_fake_glove_pub.py --stop-after 14`를 먼저 띄운다.

Python 3.10(Humble) → 3.12(Jazzy) 확인 내용: 기존 파일은 표준 라이브러리(argparse, json, math, time, pathlib),
rclpy, 표준 메시지, dynamixel_sdk만 import하며 3.12에서 모두 컴파일된다. controller는 apt `dynamixel_sdk` 4.0.3으로
import와 `--help`까지 확인했다(포트는 열지 않음). Jazzy에서 Ctrl+C는 `KeyboardInterrupt`로 전달되어 controller의
`finally` → `cleanup()`(torque OFF)이 실행된다. 그 뒤 `rclpy.shutdown()`이 "already called" 오류를 찍을 수 있지만
torque OFF 이후라 동작에는 영향이 없다. 카메라 publisher는 mediapipe가 없어 이 PC에서 실행하지 않는다.

## 7. 실물 전에 확인할 것

- 실제 장갑으로 캘리브레이션을 기록하고, 여러 번 반복했을 때 ratio가 재현되는지 확인
- 실제 60 Hz 스트림의 메시지 손실(`A message was lost!!!`)이 0.3 s stale 판정을 유발하는지
- 엄지: 굴곡 ratio만으로 LEAP 엄지(opposition)가 쓸 만한지. 벌림 정보는 `r_thumb_brake`(−Z)에 있음
- 현재 구성은 오른손 장갑 → 왼손 LEAP. 굴곡 ratio는 좌우 공통이지만 오른손 LEAP에는 별도 motor map 필요
- `hand` 모드의 포트 확인·controller 실행은 실물 없이 실행하지 않았음(U2D2 미연결)
- controller에 stale 감시가 없어서, publisher 노드가 비정상 종료하면 마지막 자세가 유지됨
