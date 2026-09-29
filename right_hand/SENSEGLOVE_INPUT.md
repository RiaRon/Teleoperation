# 오른손 Nova 2 입력 → finger ratio

오른손 장갑(#00782)의 `JointState`를 `/right_hand/finger_ratios`로 바꾸는 부분이다.
controller 쪽은 `README.md`를 본다. 같은 변환 코드가 `../left_hand/`에 왼손 장갑용(`l_`)으로 한 벌 더 있다.

## 1. 파일

| 파일 | 내용 |
|---|---|
| `senseglove_finger_ratio.py` | ROS 없는 변환 로직: 이름 기반 추출, 검증, 정규화, 합성, stale watchdog, 출력 제한 |
| `senseglove_finger_ratio_pub.py` | ROS 2 노드. 기본 입력 `/senseglove/glove00782/rh/joint_states`, 출력 `/right_hand/finger_ratios`. 기본 **dry-run**(publisher 없음), `--publish`일 때만 발행 |
| `senseglove_calibration_anatomical.json` | 캘리브레이션 전 기본값(SenseGlove SDK v22 해부학 한계, `r_`). 실측 샘플이 아님 |
| `senseglove_record_calibration.py` | open/fist를 각 2 s 기록해 중앙값으로 `senseglove_calibration.json` 생성 |
| `senseglove_fake_glove_pub.py` | 가짜 오른손 장갑(기록 자세 재생, 관절 순서 무작위, 끊김/NaN 주입) |
| `tests/test_senseglove_finger_ratio.py`, `tests/fixtures/senseglove_poses.json` | 변환 테스트 44개, 오른손 장갑 실측 자세 4개(2026-09-21) |

## 2. 변환

입력: `/senseglove/glove00782/rh/joint_states` (`sensor_msgs/JointState`, 약 60 Hz)

1. `dict(zip(msg.name, msg.position))`로 **이름 기반** 조회: `r_{thumb,index,middle,ring}_{mcp,pip,dip}` 12개.
   pinky, `*_brake`, `palm_*`는 사용하지 않는다.
2. 관절마다 `n = clamp((value - open) / (closed - open), 0, 1)`로 0~1 정규화(방향이 반대여도 동작).
3. 손가락 ratio = MCP/PIP/DIP 정규화 값의 가중 평균(기본은 같은 가중치).
   - Nova 2에서 손가락 DIP는 PIP를 따라간다(샘플에서 DIP ≈ 0.9 × PIP). 같은 가중치 평균은 대략
     1/3 MCP + 2/3 PIP가 되어, PIP 각도 하나로 만들던 카메라 ratio와 성격이 비슷하다.
   - 엄지는 같은 식을 쓰지만 open/closed 값은 따로 둔다. 엄지의 `mcp/pip/dip`은 실제로 CMC/MCP/IP 굴곡이다.
4. 출력 `[thumb, index, middle, ring]`에 `max_ratio`(노드 기본 0.7, 스크립트 0.5)와 `max_step`(0.05/publish)을
   적용해 15 Hz로 발행한다.

SenseGlove 관절값 참고(`senseglove_ros` 소스 확인): `*_mcp/pip/dip` = `HandPose::GetHandAngles()` 관절 1~3의
굴곡(Y)이고, `*_brake`의 위치값은 첫 관절 벌림(−Z)이다. 주먹 샘플이 1.571/1.745 rad에서 멈춘 것은
SDK 해부학 한계(MCP 90°, PIP 100°, DIP 90°)에 걸려 포화된 값이다.

## 3. 캘리브레이션

- 운영용 값은 **기록한 사용자 캘리브레이션**을 쓴다. 단일 스냅샷을 하드코딩하지 않는다.
- `senseglove_calibration_anatomical.json`은 기록 전 dry-run용 기본값이다. 노드가 경고를 출력한다.

| 샘플 자세 | 해부학 기본값 출력 [T, I, M, R] |
|---|---|
| open | 0.11, 0.03, 0.02, 0.19 |
| fist | 0.55, 0.87, 1.00, 1.00 |
| index_bend | 0.11, 0.73, 0.40, 0.39 |
| pinch | 0.41, 0.53, 0.31, 0.32 |

## 4. 안전 처리

| 상황 | 처리 |
|---|---|
| 필요한 joint 누락, name/position 길이 불일치, 중복 이름 | 메시지 거부, ratio 갱신 안 함 |
| NaN/Inf, `|value| > max_abs_joint_rad`(π) | 메시지 거부 |
| 유효한 메시지가 `--stale-timeout`(0.3 s) 이상 없음(끊김 또는 거부만 연속) | target = open, `max_step`으로 서서히 open 복귀 |
| 첫 메시지 전 | open(0) 출력 |
| 노드 종료(Ctrl+C) | `--publish`일 때 open 명령 1회 발행 |
| dry-run(기본) | publisher 자체를 만들지 않음 |
| 발행 속도 | 15 Hz |
| 거부 로그 | `--log-period`마다 1회로 제한 |

ratio 노드가 죽는 경우는 오른손 controller의 0.5 s watchdog(torque OFF)이 잡는다(`README.md` 5절).

## 5. 실물 전에 확인할 것

- 실제 장갑으로 캘리브레이션을 기록하고, 반복했을 때 ratio가 재현되는지
- 실제 60 Hz 스트림의 메시지 손실(`A message was lost!!!`)이 0.3 s stale 판정을 유발하는지
- 엄지: 굴곡 ratio만으로 LEAP 엄지(opposition)가 쓸 만한지. 벌림 정보는 `r_thumb_brake`(−Z)에 있음
