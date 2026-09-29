# Teleoperation: SenseGlove Nova 2 → LEAP Hand

SenseGlove Nova 2 장갑으로 LEAP Hand를 움직이는 텔레오퍼레이션 저장소다.
**왼손끼리, 오른손끼리** 따로 연결하며, 각 폴더는 독립적으로 실행·테스트된다.

```
Teleoperation/
├── left_hand/    왼손 Nova 2 (#00795) → 왼손 LEAP (연구실 기존 손, 처음 받은 카메라 코드 포함)
└── right_hand/   오른손 Nova 2 (#00782) → 오른손 LEAP Hand V1 (신규 controller)
```

```
left_hand : /senseglove/glove00795/lh/joint_states → senseglove_finger_ratio_pub.py → /left_hand/finger_ratios
            → left_hand_finger_ratio_node_safe.py  ◄── /left_hand/arm
right_hand: /senseglove/glove00782/rh/joint_states → senseglove_finger_ratio_pub.py → /right_hand/finger_ratios
            → right_hand_finger_ratio_controller.py ◄── /right_hand/arm
```

- 실행 환경: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 (`sudo apt install ros-jazzy-dynamixel-sdk`)
- **실물 LEAP Hand 모터는 아직 한 번도 구동하지 않았다.** 모든 검증은 테스트와 dry-run이다.
- SenseGlove 변환 코드(`senseglove_*.py`)는 두 폴더에 한 벌씩 있다. 폴더별로 기본 장갑 토픽, 관절 접두사(`l_`/`r_`),
  출력 토픽만 다르다.

| 폴더 | 상태 | 문서 |
|---|---|---|
| `left_hand/` | 왼손 장갑 → 왼손 LEAP dry-run 완료. 왼손 장갑 실데이터·캘리브레이션 미확인 | `left_hand/README.md` |
| `right_hand/` | dry-run 완료. 오른손 LEAP close 자세·port·baudrate가 TBD라 hardware 모드 차단 | `right_hand/README.md`, `right_hand/SENSEGLOVE_INPUT.md` |

## 테스트

```bash
source /opt/ros/jazzy/setup.bash
cd left_hand  && python3 -m pytest tests -q    # 58 passed
cd ../right_hand && python3 -m pytest tests -q # 101 passed (left_hand 없이 단독이면 98 passed, 3 skipped)
```

## 커밋 구성

1. `Import original left_hand_project` = 처음 받은 `left_hand_project.zip` 원본 그대로 (`.venv`, `__pycache__`만 제외)
2. SenseGlove 입력과 오른손 LEAP controller 추가
3. `left_hand/`, `right_hand/`로 분리, 오른손 장갑 → 왼손 LEAP 연결 삭제, 왼손 장갑 → 왼손 LEAP 추가

## 처음 받은 코드에서 바뀐 것

### 원본 파일: 모두 `left_hand/`로 이동, 수정은 1개

| 파일 | 변경 |
|---|---|
| `left_hand/left_hand_finger_ratio_node_safe.py` | `finger_callback`에 NaN/Inf 검사 추가(8줄). 원래 `clamp()`는 `max(0, min(1, x))`라서 NaN이 1.0(주먹 명령)으로 바뀌었다. 이제 NaN/Inf가 섞인 메시지는 경고 후 무시한다 |

나머지 원본(카메라 publisher, 왼손 motor map, URDF, `dxl_*` 도구, `pose_preview.py`, Humble용 `.sh`, `backup/`)은
위치만 `left_hand/`로 옮겼고 내용은 **바이트 단위로 그대로**다. 원본 `.sh`는 Humble과 `~/left_hand_project` 경로를
가정하므로 이 구조·환경에서는 동작하지 않는다(Jazzy용 스크립트를 새로 추가함).

### 원본에서 뺀 것

- `.venv/` (Python 3.10 전용 바이너리, 약 340 MB): Jazzy/Python 3.12에서 쓸 수 없음. `dynamixel_sdk`는 apt로 대체
- `__pycache__/`

### 추가: 왼손 장갑 → 왼손 LEAP (`left_hand/`)

| 파일 | 역할 |
|---|---|
| `senseglove_finger_ratio.py` | ROS 없는 변환: joint 이름 기반 추출, 검증(누락/길이/중복/NaN/Inf/±π), 관절별 open/closed 정규화 → 손가락별 평균, 0.3 s stale watchdog(끊기면 open으로 램프), 출력 상한·변화량 제한 |
| `senseglove_finger_ratio_pub.py` | ROS 2 노드. 입력 `/senseglove/glove00795/lh/joint_states`, 출력 `/left_hand/finger_ratios`, 15 Hz. 기본 dry-run |
| `senseglove_calibration_anatomical.json` | 캘리브레이션 전 기본값(`l_`, SenseGlove SDK 해부학 한계) |
| `senseglove_record_calibration.py`, `senseglove_fake_glove_pub.py` | 왼손 장갑 캘리브레이션 기록, 가짜 왼손 장갑 |
| `start_left_hand_senseglove_system.sh`, `LEAP-Left-Arm-Jazzy.sh`, `LEAP-Left-Disarm-Jazzy.sh` | Jazzy용 실행(`dryrun`/`preview`/`hand`), ARM/DISARM |
| `tests/` | SenseGlove 변환(`l_`) 44개, controller NaN/Inf 수정 14개(원본에서는 12개 실패) |
| `README.md` | 원본 카메라 구조 분석, 왼손 장갑 연결, 실행법 |

### 추가: 오른손 장갑 → 오른손 LEAP Hand V1 (`right_hand/`)

| 파일 | 역할 |
|---|---|
| `senseglove_*.py`, `senseglove_calibration_anatomical.json` | 위와 같은 변환(`r_`), 입력 `/senseglove/glove00782/rh/joint_states`, 출력 `/right_hand/finger_ratios` |
| `config/right_hand.json` | 오른손 설정. ID·open 자세(180°)·관절 한계는 공식 V1 자료 근거, close 자세·port·baudrate는 TBD, 항목별 `hardware_verified` |
| `right_hand_controller_core.py` | ROS·SDK 없는 로직: 설정 검증, 차단 조건, ratio 검증, 보간, 한계 clamp, ARM 게이트, 시작 램프, 0.5 s watchdog(torque OFF), 종료 처리 |
| `right_hand_dynamixel_bus.py` | 실제 Dynamixel I/O (hardware 모드에서만 `dynamixel_sdk` import) |
| `right_hand_finger_ratio_controller.py` | ROS 2 노드 `/right_hand/finger_ratios`, `/right_hand/arm`. 기본 dry-run |
| `right_hand_hw_check.py`, `right_hand_torque_off.py` | 읽기 전용 실물 점검·open/close 기록, controller가 없을 때 직접 torque OFF |
| `LEAP-Right-Arm-Jazzy.sh`, `LEAP-Right-Disarm-Jazzy.sh`, `start_right_hand_senseglove_system.sh` | 오른손 ARM/DISARM, 실행 스크립트 |
| `tests/` | SenseGlove 변환(`r_`) 44개, 오른손 controller·통합 57개, 합성 테스트 전용 설정 |
| `README.md`, `SENSEGLOVE_INPUT.md` | 오른손 값 근거, 안전 정책, 실물 체크리스트, 첫 구동 절차 / 오른손 장갑 변환 |

### 삭제한 것

- **오른손 장갑(#00782) → 왼손 LEAP 연결.** 커밋 2에서는 오른손 장갑이 `/left_hand/finger_ratios`로 왼손 LEAP을
  움직이도록 되어 있었다. 이제 오른손 장갑은 `/right_hand/finger_ratios`(오른손 LEAP)로만, 왼손 LEAP은 왼손 장갑으로만 연결한다.
