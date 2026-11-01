# 비전 위험 판정 히스테리시스 · 급접근 주의 신호 · Pose 부하 · 모터 급발진 감시

작성 2026-11-01.

## 1. 기존 코드에 히스테리시스가 있었나

| 위치 | 있음/없음 | 내용 |
|---|---|---|
| `robot_safety/lidar_obstacle_node.py` | 있음(LiDAR) | 정지 0.8 m 즉시, 해제는 1.0 m 안에 점이 없는 스캔 3회 연속 |
| `robot_vision/detector_node.py` | 없음 | 매 추론 결과를 그대로 발행(빈 배열 포함). ByteTrack ID, 1 s 공백이면 추적기 리셋 |
| `robot_vision/pose_node.py` | 없음 | detector 와 같은 코드(pose 모드), 기본 비활성 |
| `robot_safety/safety_manager_node.py` | 비전 입력 없음 | LiDAR/E-stop/명령만 사용 |

→ 사람 감지·위험 판정에는 히스테리시스가 없었다. 새로 `robot_vision/hazard_monitor_node` 를 추가했다.

## 2. hazard_monitor_node (새로 추가)

* 핵심 로직: `robot_vision/robot_vision/person_hazard.py` (ROS 없이 시험 가능, 시험 16개)
* 입력: `/vision/detections`(사람 클래스), `/camera/camera_info`(영상 크기), `/odom` → 없으면 `/cmd_vel_safe`
  (로봇 자신의 움직임), 선택 `/vision/poses`
* 출력: `/vision/person_hazard`(Bool), `/vision/hazard_state`(JSON)
* 설정: `robot_vision/config/vision.yaml` 의 `hazard_monitor_node` 블록

| 규칙 | 설정 | 동작 |
|---|---|---|
| 진입 | `min_score`, `path_zone`, `near_height_enter`, `near_foot_y_enter` | 한 프레임이라도 조건 충족 시 **그 프레임에서 바로** HAZARD. 연속 관측 조건을 진입에 쓰지 않는다 |
| 해제 | `clear_frames` + `clear_sec` (둘 다) | 완화된 해제 조건(`path_zone_exit_margin`, `near_*_exit`)을 만족하는 사람이 없는 프레임이 연속으로 쌓여야 CLEAR |
| 관측 누락 유예 | `miss_grace_sec` | 위험 판정 대상이 검출에서 사라지면 그 시간 동안은 있다고 봄. 화면 안에서 영역 밖으로 나간 것이 관측되면 유예 없음 |
| 추적 ID 변경 | `id_switch_iou`, `id_switch_max_gap_sec` | 전역 상태는 ID 와 무관. 사라진 ID 와 많이 겹치는 새 ID 는 이어 붙이고 `ID_SWITCH` 로그 |
| 입력 단절 | `input_timeout_sec`, `stale_is_hazard` | 검출 메시지가 끊기면 STALE, 기본은 위험으로 취급. 재개 후에도 해제 조건을 다시 채워야 함 |
| 시간 순서 | — | stamp 가 뒤로 가면 버리고 `OUT_OF_ORDER` 기록 |
| 진단 로그 | `heartbeat_period_sec` | 상태 변화(사유·트랙·점수·발 위치), ID 변경, 단절, 주의 경보, 카운터(`raw_toggles` vs `state_changes`) |

`raw_toggles`(원시 판정이 바뀐 횟수)와 `state_changes`(출력이 바뀐 횟수)를 같이 남겨 흔들림을 얼마나
걸렀는지 볼 수 있다.

### Safety 연결 (선택, 기본 끔)
`robot_safety/config/safety.yaml` 의 `use_vision_hazard: true` 일 때만 Safety 가 `/vision/person_hazard` 를
구독한다. True 를 받은 **그 콜백에서 바로** 0 을 발행하고(`STOP:vision_hazard`), 입력이 0.5 s 끊기면
`STOP:vision_hazard_timeout`. 해제 후에는 다른 정지와 같이 새 명령을 기다린다. LiDAR/E-stop 판단을 대신하지
않고 정지 사유만 추가한다. 시험 5개(`robot_safety/test/test_vision_hazard_input.py`).

**아직 끄는 이유**: 경로 영역·크기 기준이 미보정 화면 값이고, Pi 에서 검출 주기가 측정되지 않았다.
실물에서 `/vision/hazard_state` 로 오탐/놓침을 먼저 기록한 뒤 켠다.

## 3. "급발진 탐색" = 보행자가 갑자기 경로로 뛰어드는 상황 (화면상 주의 신호)

같은 추적 대상의 시간 순서가 맞는 관측만 사용한다(창 `approach_window_sec`, 최소 `approach_min_obs`).

* 경로 쪽 수평 속도 [영상 폭/s]: 영역 밖이면 경로 중심 방향 성분, 영역 안이면 가로지르는 속도.
  로봇 yaw 로 생기는 화면 이동은 공칭 HFOV 로 근사 보정하고, `|ω| > max_ego_yaw_rate` 이면 신뢰하지 않음.
* 확대율 d ln(h)/dt [1/s]: 로봇이 전진 중이면 정지 물체도 커지므로 `|v| ≤ max_ego_speed_for_expansion`
  일 때만 사용.
* 회귀 결정계수(R²), 평균 점수, 트랙 확인 관측 수, 화면 좌우 끝 잘림을 신뢰도 조건으로 사용.
* 결과: NONE / WATCH / ALERT (`attention_hold_sec` 동안 유지), `/vision/hazard_state` 의 트랙별 `cues`.

**하지 않는 것**: 미터 단위 속도나 충돌 시간(TTC)을 계산하지 않는다. 확대율의 역수는 이론상 거리 없이 쓰는
접근 시간 근사지만, bbox 높이 잡음·머리 잘림·자세 변화·차체 흔들림 때문에 안전 판단 값으로 쓰지 않는다.
주의 신호는 Safety 입력이 아니다. Pose 키포인트는 선택적으로 발 위치 보정에만 쓰고 판정 근거로 단독 사용하지 않는다.

## 4. Pose 노드가 꺼져 있는 이유와 Pi 부하

* 꺼진 이유(코드): `vision.yaml`/`robot.yaml` 의 `pose_node.enabled: false`, 주석 "객체 탐지의 실물 부하를
  먼저 측정한 뒤 활성화". 결함 때문이 아니라 부하 측정 전 정책이다. Pose 와 탐지의 추적 ID 는 서로 독립이다.
* 사용자 WSL 에는 ultralytics 가 설치돼 있지 않아(스냅샷 `env.txt`) 탐지·Pose 모두 현재 모델을 못 올린다.
* 공개 수치(Raspberry Pi 5, Ultralytics 문서, 640 입력, 전·후처리 제외): YOLO26n PyTorch 약 299 ms,
  ONNX 약 126 ms, NCNN 약 67 ms / 이미지. YOLO11n 은 ONNX 6.79 FPS. 같은 세대 비교에서 Pose 모델은
  FLOPs 5.5 B → 7.6 B (약 1.4 배).
* **추정(측정 아님)**: 현재 설정(PyTorch `.pt`, imgsz 416 ≈ 640 의 42% 픽셀)이면 탐지 약 0.1~0.15 s/회,
  Pose 약 0.15~0.2 s/회. 탐지 5 Hz + Pose 2 Hz 를 동시에 돌리면 추론만으로 Pi 5 의 4 코어 중 약 2 코어가
  계속 쓰이는 수준이라 Nav2·AMCL·QR 과 경합할 가능성이 크다. NCNN 변환 시 약 1/4~1/2 로 줄어들 것으로 본다.
* Pi 에서 측정하는 방법:
  ```bash
  # 탐지만 → 탐지+Pose 순서로, 각각 1~2 분
  ros2 launch robot_bringup robot.launch.py            # camera_node use_camera:=true 로 실물 카메라
  ros2 param get /detector_node inference_rate_hz
  # heartbeat 의 inference_ms, received/published/dropped 확인
  pidstat -u -p $(pgrep -d, -f "detector_node|pose_node|camera_node") 5 12
  ```
  판단 기준 예: 탐지 `inference_ms × inference_rate_hz < 0.6`, 전체 CPU 70 % 이하, 카메라 fps 유지.

## 5. 모터 급발진(명령과 실제 움직임 불일치) — 별도 안전 기능 제안

보행자 급접근과 별개로 정의한다: **안전 명령(`/cmd_vel_safe`)과 실제 움직임(바퀴 엔코더 `/wheel/odom`,
IMU `/imu/data`)이 어긋나는 상태.**

| 감시 항목 | 조건 예(초기값, 실물 조정) | 의미 |
|---|---|---|
| 명령 없는 움직임 | 명령 0 인데 `|v_wheel| > 0.05 m/s` 또는 `|ω_imu| > 0.15 rad/s` 가 0.3 s 이상 | 급발진·브레이크 실패·경사 미끄러짐 |
| 과속 | `|v_wheel| > |v_cmd| + 0.1` 또는 `> 1.2·max` 가 0.2 s 이상 | 드라이버 폭주 |
| 방향 불일치 | 명령과 측정의 부호 반대, 0.3 s 이상 | 배선/설정 오류 |
| 엔코더·IMU 불일치 | 회전 중 `|ω_wheel − ω_imu|` 가 기준 초과 | 미끄럼(스키드는 정상 범위가 큼) 또는 센서 고장 |
| 피드백 끊김 | 엔코더/IMU 메시지 단절 | 감시 불능 = 정지 |

* 대응: 즉시 정지 + **래치**(자동 해제 금지, 운영자 확인 후 해제), 원인 로그.
* 위치: 1차는 **STM32 펌웨어**(명령 단절 워치독, 전류·속도 한계, 엔코더 기반 과속 차단)와 하드웨어 E-stop
  (전원 차단). ROS 쪽 감시는 2차다 — 모터가 명령을 무시하는 고장에서는 ROS 가 0 을 보내도 멈추지 않으므로
  ROS 감시만으로는 충분하지 않다.
* 필요 여부: **필요하다.** 35 kg 로봇에 인휠 모터 4개이고, 실물 `/estop_state` 발행·모터 bridge 가 아직
  없으므로 bridge 를 만들 때 같이 정의하는 것이 맞다.
* 지금 할 수 있는 것: `robot_bringup motion_chain_monitor` 가 관찰 전용으로 `UNCOMMANDED_MOTION` 이벤트를
  기록한다(정지시키지 않음). 시뮬레이션·벤치에서 기준값을 정하는 데 쓴다.
