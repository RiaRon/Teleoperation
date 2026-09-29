# 왼손 Nova 2 → 왼손 LEAP Hand

```
Left Nova 2 (#00795) → /senseglove/glove00795/lh/joint_states (sensor_msgs/JointState)
  → senseglove_finger_ratio_pub.py : l_ MCP/PIP/DIP → [thumb, index, middle, ring] 0~1, 15 Hz
  → /left_hand/finger_ratios (std_msgs/Float32MultiArray)
  ├→ left_hand_finger_joint_state_pub.py → /joint_states → RViz (preview)
  └→ left_hand_finger_ratio_node_safe.py ◄── /left_hand/arm (std_msgs/Bool) → 연구실 왼손 LEAP (Dynamixel 16개)
```

- 카메라(MediaPipe) 입력 대신 **왼손 장갑**으로 기존 왼손 LEAP controller를 움직인다.
- 이전에 있던 "오른손 장갑(#00782) → 왼손 LEAP" 연결은 삭제했다. 오른손 장갑은 `../right_hand/`에서 오른손 LEAP에만 연결한다.
- **아직 실물로 확인한 것이 없다.** 왼손 장갑의 실제 데이터 수신, 캘리브레이션, 왼손 LEAP 구동 모두 미확인이다.

## 1. 기존 카메라 기반 구조 (처음 받은 코드)

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

- 원본 실행 스크립트(`start_left_hand_camera_*.sh`, `LEAP-Left-{Arm,Disarm,Fist,VSign}.sh`, `backup/*.sh`)는
  ROS 2 Humble, Python 3.10 `.venv`, `~/left_hand_project` 경로를 가정한다. 이 PC(Jazzy)에서는 동작하지 않으며,
  원본 보존을 위해 수정하지 않았다.
- `start_left_hand_camera_stable_system.sh`는 `backup/`에만 있는 `left_hand_finger_ratio_node.py`를 실행한다.
- controller에는 입력 끊김 감시가 없다. 입력이 끊기면 마지막 목표 자세를 유지하므로, open 복귀는 ratio 노드 책임이다.

## 2. 이 폴더의 파일

| 파일 | 내용 |
|---|---|
| 원본: `left_hand_*.py`, `left_hand_motor_map.json`, `left_hand_simple.urdf`, `dxl_*.py`, `pose_preview.py`, Humble용 `.sh`, `backup/` | 처음 받은 코드. `left_hand_finger_ratio_node_safe.py`의 NaN/Inf 검사 외에는 그대로 |
| `senseglove_finger_ratio.py` | ROS 없는 변환 로직(오른손 폴더와 같은 코드) |
| `senseglove_finger_ratio_pub.py` | ROS 2 노드. 기본 입력 `/senseglove/glove00795/lh/joint_states`, 출력 `/left_hand/finger_ratios`. 기본 dry-run |
| `senseglove_calibration_anatomical.json` | 캘리브레이션 전 기본값. `joint_prefix: "l_"`, SDK 왼손 해부학 한계 |
| `senseglove_record_calibration.py` | 왼손 장갑 open/fist 기록 → `senseglove_calibration.json` |
| `senseglove_fake_glove_pub.py` | 가짜 왼손 장갑(`l_`, `/senseglove/glove00795/lh/joint_states`) |
| `start_left_hand_senseglove_system.sh` | `dryrun` / `preview` / `hand` |
| `LEAP-Left-Arm-Jazzy.sh`, `LEAP-Left-Disarm-Jazzy.sh` | Jazzy용 ARM / DISARM |
| `tests/` | 58개: SenseGlove 변환(왼손 `l_` 기준) 44개, controller NaN/Inf 수정 14개 |

## 3. 왼손 장갑 → finger ratio

1. `dict(zip(msg.name, msg.position))`로 **이름 기반** 조회: `l_{thumb,index,middle,ring}_{mcp,pip,dip}` 12개.
   pinky, `*_brake`, `palm_*`는 쓰지 않는다. 오른손 이름(`r_`)만 있는 메시지는 "필요한 관절 누락"으로 거부된다.
2. 관절마다 `(value − open) / (closed − open)`으로 0~1 정규화, 손가락별 세 관절 평균(가중치 동일).
3. 출력에 `max_ratio`(스크립트 0.5)와 `max_step`(발행당 0.05)을 적용해 15 Hz로 발행.

왼손에 같은 코드를 쓰는 근거:

- `senseglove_ros`의 `nova2_left.yaml`은 `nova2_right.yaml`과 접두사(`l_`/`r_`)만 다르다(관절 이름·순서 동일).
  `senseglove_bringup/config/gloves.yaml`에 왼손 nova2 serial `00795`가 등록되어 있다.
- SenseGlove SDK v22로 계산한 결과, 왼손과 오른손은 **굽힘(mcp/pip/dip) 값의 부호와 해부학 한계가 같고**,
  첫 관절 벌림(`*_brake` 위치값)만 부호가 반대다. 벌림은 쓰지 않는다.

  | SDK 자세 | 오른손 index MCP/PIP/DIP | 왼손 index MCP/PIP/DIP |
  |---|---|---|
  | FlatHand | 0.017 / 0.017 / 0.017 | 0.017 / 0.017 / 0.017 |
  | Fist | 1.396 / 1.745 / 1.571 | 1.396 / 1.745 / 1.571 |

- 실제 왼손 장갑에서 같은지는 **아직 확인하지 않았다**. 테스트 fixture와 가짜 장갑의 자세는 오른손 장갑(#00782)
  실측값을 `l_` 이름으로 바꿔 쓴 것이다.

## 4. 안전 처리

| 상황 | 처리 |
|---|---|
| 필요한 joint 누락, name/position 길이 불일치, 중복 이름 | 메시지 거부 |
| NaN/Inf, ±π 초과 | 메시지 거부 |
| 유효한 메시지가 0.3 s 이상 없음 | open으로 서서히 복귀 |
| 첫 메시지 전 / 노드 종료 | open(0) 출력 / open 1회 발행 |
| dry-run(기본) | publisher를 만들지 않음 |
| 발행 속도 | 15 Hz (controller가 메시지마다 57600 baud로 16개 모터에 씀) |
| controller NaN | `clamp(NaN)`이 1.0(주먹)이던 문제를 수정, NaN/Inf 메시지 무시 |

`left_hand_finger_joint_state_pub.py`의 `clamp_ratio`는 NaN을 통과시킨다(RViz 표시 전용, 원본 유지).

## 5. 실행 (Ubuntu 24.04 + ROS 2 Jazzy)

준비: `sudo apt install ros-jazzy-dynamixel-sdk`. SenseCom에서 왼손 장갑 연결, senseglove bringup 실행.

```bash
source /opt/ros/jazzy/setup.bash && cd ~/Teleoperation/left_hand
python3 -m pytest tests -q
ros2 topic hz /senseglove/glove00795/lh/joint_states     # 0. 왼손 장갑 데이터 수신 확인
python3 senseglove_record_calibration.py                 # 1. 왼손 장갑 캘리브레이션
./start_left_hand_senseglove_system.sh                   # 2. dryrun: ratio 로그 확인
./start_left_hand_senseglove_system.sh preview           # 3. RViz 모델 확인 (모터 없음)
./start_left_hand_senseglove_system.sh hand              # 4. 실물 승인 후: 손을 편 상태에서 ./LEAP-Left-Arm-Jazzy.sh
```

장갑 없이 확인: `python3 senseglove_fake_glove_pub.py --stop-after 14`를 먼저 띄운다.

## 6. 실물 전에 확인할 것

- 왼손 장갑(#00795) 연결, `/senseglove/glove00795/lh/joint_states`의 실제 이름·주기
- 왼손 장갑에서 손을 쥐면 mcp/pip/dip 값이 **증가**하는지(SDK 기준과 같은 부호인지)
- 왼손 캘리브레이션 기록 후 반복 재현성, 메시지 손실이 0.3 s stale을 유발하는지
- 왼손 LEAP U2D2 포트(`PORT=/dev/ttyUSB0` 기본), `hand` 모드 controller 실행
- controller에 watchdog이 없어서 ratio 노드가 비정상 종료하면 마지막 자세가 유지됨
