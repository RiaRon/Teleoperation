# Teleoperation: SenseGlove Nova 2 → LEAP Hand V1

기존 카메라(MediaPipe) 기반 LEAP Hand 손가락 추종 코드에 **SenseGlove Nova 2 장갑 입력**을 붙이고,
**오른손 LEAP Hand V1 controller**를 별도로 추가한 작업 저장소다.

```
SenseGlove Nova 2 오른손 (#00782)
  → /senseglove/glove00782/rh/joint_states (sensor_msgs/JointState, 약 60 Hz)
  → senseglove_finger_ratio_pub.py : MCP/PIP/DIP → [thumb, index, middle, ring] 0~1, 15 Hz
  ├→ /left_hand/finger_ratios  → left_hand_finger_ratio_node_safe.py  (기존 왼손 LEAP)
  └→ /right_hand/finger_ratios → right_hand_finger_ratio_controller.py (신규 오른손 LEAP V1)
```

- 실행 환경: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 (`sudo apt install ros-jazzy-dynamixel-sdk`)
- **실물 LEAP Hand 모터는 아직 한 번도 구동하지 않았다.** 모든 검증은 테스트와 dry-run이다.
- 오른손은 close 자세·port·baudrate가 TBD라 hardware 모드가 막혀 있다(`README_RIGHT_HAND.md`).

## 커밋 구성

1. **첫 커밋 = 처음 받은 `left_hand_project.zip` 원본 그대로** (`.venv`, `__pycache__`만 제외)
2. 두 번째 커밋 = 이후 작업 전체

GitHub에서 두 커밋을 비교하면 원본 대비 모든 변경을 볼 수 있다.

## 처음 받은 코드에서 바뀐 것

### 원본 파일 중 수정한 것: 1개

| 파일 | 변경 |
|---|---|
| `left_hand_finger_ratio_node_safe.py` | `finger_callback`에 NaN/Inf 검사 추가(`import math` 포함 8줄). 원래 `clamp()`는 `max(0, min(1, x))`라서 NaN이 1.0(주먹 명령)으로 바뀌었다. 이제 NaN/Inf가 섞인 메시지는 경고 후 무시한다. 그 외 동작은 동일 |

나머지 원본 파일(카메라 publisher, 왼손 motor map, URDF, `dxl_*` 도구, Humble용 `.sh`, `backup/`)은 **바이트 단위로 그대로**다.

### 원본에서 뺀 것

- `.venv/` (Python 3.10 전용 바이너리, 약 340 MB) → Jazzy/Python 3.12에서 쓸 수 없어 제외. `dynamixel_sdk`는 apt 패키지로 대체
- `__pycache__/`

### 추가한 것: SenseGlove 입력 (2026-09-26)

| 파일 | 역할 |
|---|---|
| `senseglove_finger_ratio.py` | ROS 없는 변환 로직: joint 이름 기반 추출, 검증(누락/길이/중복/NaN/Inf/±π), 관절별 open/closed 정규화 → 손가락별 평균, 0.3 s stale watchdog(끊기면 open으로 램프), 출력 상한·변화량 제한 |
| `senseglove_finger_ratio_pub.py` | ROS 2 노드. 기본 dry-run(publisher 없음), `--publish` 시 15 Hz 발행, `--output-topic`으로 왼손/오른손 선택 |
| `senseglove_record_calibration.py` | 사용자 open/fist 자세를 각 2 s 기록해 중앙값으로 `senseglove_calibration.json` 생성 |
| `senseglove_calibration_anatomical.json` | 캘리브레이션 전 기본값(SenseGlove SDK 해부학 한계, 실측 아님) |
| `senseglove_fake_glove_pub.py` | 장갑 없이 테스트하는 가짜 장갑(기록 자세 재생, 끊김·NaN 주입) |
| `start_left_hand_senseglove_system.sh` | 왼손용 Jazzy 실행 스크립트 `dryrun` / `preview` / `hand` |
| `LEAP-Left-Arm-Jazzy.sh`, `LEAP-Left-Disarm-Jazzy.sh` | 원본 `LEAP-Left-*.sh`는 Humble 경로를 불러와 Jazzy PC에서 실패하므로 Jazzy 버전 추가 |
| `README_SENSEGLOVE.md` | 기존 카메라 구조 분석, 변환 방식, 안전 처리, 실행법 |

### 추가한 것: 오른손 LEAP Hand V1 controller (2026-09-28)

| 파일 | 역할 |
|---|---|
| `config/right_hand.json` | 오른손 설정. ID·open 자세(180°)·관절 한계는 공식 V1 자료 근거, close 자세·port·baudrate는 TBD, 항목별 `hardware_verified` |
| `right_hand_controller_core.py` | ROS·SDK 없는 로직: 설정 검증, 차단 조건, ratio 검증, 보간, 한계 clamp, ARM 게이트, 시작 램프, 0.5 s watchdog(torque OFF), 종료 처리 |
| `right_hand_dynamixel_bus.py` | 실제 Dynamixel I/O (hardware 모드에서만 `dynamixel_sdk` import) |
| `right_hand_finger_ratio_controller.py` | ROS 2 노드 `/right_hand/finger_ratios`, `/right_hand/arm`. 기본 dry-run |
| `right_hand_hw_check.py` | 읽기 전용 실물 점검(baud 탐색, ID ping, 운영 모드, present 각도, open/close 기록) |
| `right_hand_torque_off.py` | controller가 없을 때 직접 torque OFF |
| `LEAP-Right-Arm-Jazzy.sh`, `LEAP-Right-Disarm-Jazzy.sh`, `start_right_hand_senseglove_system.sh` | 오른손 ARM/DISARM, 실행 스크립트 |
| `README_RIGHT_HAND.md` | 왼손 controller 분석, 오른손 값 근거, 안전 정책, 실물 체크리스트, 첫 구동 절차 |

### 테스트

`tests/` 전체 115개(ROS 환경 필요):

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest tests -q
```

- `test_senseglove_finger_ratio.py`: 장갑 → ratio 변환·검증·watchdog
- `test_controller_nan_guard.py`: 왼손 controller NaN/Inf 수정 확인(원본에서는 12개 실패)
- `test_right_hand_controller.py`, `test_right_hand_integration.py`: 오른손 controller, 가짜 장갑 → 오른손 목표값
- `tests/fixtures/right_hand_synthetic_test.json`은 **합성 테스트 전용**(close 값 임의, hardware 모드가 거부)

## 문서

- `README_SENSEGLOVE.md`: SenseGlove 입력, 왼손 연결, Jazzy 환경
- `README_RIGHT_HAND.md`: 오른손 LEAP V1 controller, 실물 전 체크리스트
