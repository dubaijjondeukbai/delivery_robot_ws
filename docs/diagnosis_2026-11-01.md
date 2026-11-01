# 2026-11-01 진단 보고: 단일 카메라 정리 · 명령 흐름 · EKF · 비전 히스테리시스

## 0. 기준 상태와 검증 환경

* 사용자 WSL(`~/delivery_robot_ws`, Ubuntu-26.04): `main` = `2f8dbaf`, 작업 트리 깨끗함(로컬 수정·stash 없음).
  ROS 2 Lyrical, Nav2 1.5.2, Gazebo(gz sim) 10.5.0, Python 3.14.4, ultralytics 미설치.
* 윈도우 미러 `C:\Users\UOS\delivery_robot_ws_mirror\src`: WSL 에 반영되지 않은 6개 파일
  (10-27 CAD 치수: `robot_params.xacro`, `robot.urdf.xacro`, `robot_description/README.md`, `nav2.yaml` footprint,
  `localization.yaml` 예비 laser TF, `safety.yaml` 주석 / 11-01 `front_camera_count`). 패치의 첫 커밋으로 보존.
* **클라우드 검증 환경은 Lyrical 이 아니다**: ROS 2 Jazzy + Nav2 1.3.13 + Gazebo Harmonic 8.10 (conda RoboStack).
  Jazzy 에는 `PathHandlerSelector` 가 없어 Lyrical 용 BT 대신 Jazzy 기본 BT 로 돌렸고, 클라우드 CPU(2 vCPU) 과부하로
  BT `default_server_timeout`(20 ms)이 터져 시험용 사본에서만 200 ms 로 올렸다. 물리·EKF·Safety·브리지·URDF 는
  사용자 코드 그대로다.

## 1. 명령 흐름 점검

```
Nav2 controller/behavior ─/cmd_vel_nav─> velocity_smoother ─/cmd_vel_auto─> safety_manager ─/cmd_vel_safe─>
  ros_gz_bridge ─/model/delivery_robot/cmd_vel─> DiffDrive ─/wheel/odom─> EKF ─/odom + TF odom→base_link
```

| 단계 | 코드 확인 | 클라우드 시뮬레이션 |
|---|---|---|
| controller/behavior → `/cmd_vel_nav` | `navigation.launch.py` remap, `enable_stamped_cmd_vel: false`(Twist) | 흐름 확인 |
| smoother → `/cmd_vel_auto` | `motion_enabled:=true` 일 때만(기본 false 는 `/cmd_vel_nav_dry_run`) | 흐름 확인 |
| Safety → `/cmd_vel_safe` | Twist, 20 Hz, 정지 사유 `/safety/state` | READY/STOP 전환 확인 |
| `sim_estop_publisher` | `sim.launch.py publish_estop`, `robot.launch.py sim_estop`(기본=sim) 연결됨 | `/estop_state=false` 10 Hz, Safety 가 대기 상태를 벗어남 |
| E-stop 동작 | `ros2 param set /sim_estop_publisher estop_active true` | `STOP:estop_active` → false 후 `waiting_for_new_command` → `READY`, 목표 성공 |
| E-stop 끊김 | `estop_timeout_sec: 1.0` | 발행 노드를 끄자 `STOP:estop_state_timeout` |
| bridge → DiffDrive → `/wheel/odom` | `bridge.yaml` | 확인 |
| EKF → `/odom`, TF | `ekf.yaml` | 확인(아래 5절의 회전 문제는 따로) |

`sim_estop_publisher`·`/safety/state` 해결은 현재 코드에서 실제로 연결돼 있으며 다시 만들지 않았다.

## 2. 정지 사유를 구분해 기록하기

새 관찰 노드 `ros2 run robot_bringup motion_chain_monitor` (구독만, 실물에서도 안전):

| 분류 | `/safety/state` / 이벤트 | 뜻 | 다음 확인 |
|---|---|---|---|
| E-stop 수신 | `STOP:waiting_for_estop_state` | 한 번도 못 받음 | 시뮬: `sim_estop` 인자, 실물: MCU bridge |
| E-stop 만료 | `STOP:estop_state_timeout` | 1 s 이상 끊김 | 발행 주기, QoS/도메인 |
| E-stop 활성 | `STOP:estop_active` | true 수신 | 버튼/파라미터 |
| 장애물 | `STOP:obstacle_detected` + `LIDAR_STOP reason=obstacle_in_stop_zone` | 전방 ±35°, 0.8 m 안 점 | 앞 0.67 m 여유 |
| 장애물(데이터) | `STOP:obstacle_detected` + `LIDAR_STOP reason=scan_timeout / stale_scan_stamp / insufficient_*` | **장애물이 아니라 스캔 지연·형식 문제** | Gazebo 실시간 비율, 스캔 주기 |
| 명령 대기 | `STOP:waiting_for_new_command` | Safety 정지 후 새 명령 없음, 또는 Nav2 출력이 dry-run | `motion_enabled:=true` 확인 |
| 명령 시간 초과 | `STOP:command_timeout` | 0.5 s 이상 명령 없음(목표 종료 후 정상) | 목표 중이면 controller 실패 로그 |
| TF/AMCL | `FINDING TF ... 실패`, `MAP_ODOM_JUMP` | 변환 없음 / 위치 추정 점프 | AMCL, `/initialpose` |
| 비전(선택) | `STOP:vision_hazard(_timeout)` | `use_vision_hazard:=true` 일 때만 | `/vision/hazard_state` |

클라우드에서 실제로 `LIDAR_STOP reason=scan_timeout` 이 전방 3~4 m 가 비어 있는데도 나왔다(과부하로 스캔 지연).
`/safety/state` 만 보면 장애물로 보이므로, 세부 사유는 모니터의 `LIDAR_STOP` 으로 구분한다.

## 3. "Resulting plan has 0 poses"(103) 와 "Failed to make progress"(105) 는 다른 실패

| | 103 INVALID_PATH | 105 FAILED_TO_MAKE_PROGRESS |
|---|---|---|
| 위치 | controller_server `FeasiblePathHandler::transformLocalPlan` | controller_server progress checker |
| 의미 | 전역 경로의 가장 가까운 점이 로컬 코스트맵 창(6 m) 밖 → 변환된 경로가 비어 있음 | 경로는 있고 명령도 나가는데 10 s 동안 0.15 m 또는 0.1 rad 이상 못 움직임 |
| 흔한 원인 | 위치 추정 점프(AMCL 재초기화) | Safety 정지(E-stop, 장애물, 스캔 지연), 로봇이 명령만큼 안 움직임(스키드 회전), 시간 비율 |
| 사용자 BT | 재계획 후 FollowPath 재시도(2f8dbaf 변경) | 로컬 코스트맵 비우고 재시도 → Spin/Wait 복구 |
| 모니터 | `INVALID_PATH_0_POSES` + 직전 `INITIALPOSE`/`MAP_ODOM_JUMP` | `FAILED_TO_MAKE_PROGRESS` + 그 시각의 `/safety/state`, 회전 비교 |

## 4. 목표 실행 중 2D Pose Estimate 를 다시 넣으면

* `/initialpose` 는 목표나 경로를 취소하지 않는다. AMCL 이 입자를 새 위치로 다시 뿌리고 map→odom 이 점프한다.
* Lyrical 코드(nav2 `lyrical` 브랜치)와 사용자 BT 기준:
  * 남은 경로가 4 m 이상이면 BT 가 1 Hz 로 다시 계획한다(`IsGoalNearby` 실패) → 새 위치에서 새 경로.
  * 4 m 미만이고 남은 경로가 충돌 없이 유효하면(`ValidatePath`) **기존 경로를 그대로 쓴다.** controller 는 기존
    경로에서 가장 가까운 점(앞쪽 3 m 이내 탐색)부터 따라간다. 점프가 로컬 창 밖이면 103 → 재계획.
* 클라우드(Jazzy 기본 BT, 1 Hz 무조건 재계획)에서 1.2 m 틀린 위치를 넣었더니 map→odom 1.175 m 점프 후 목표는
  "성공"했지만, 실제 위치와 추정 위치 차이가 **1.12 m 그대로 남았다**(AMCL `recovery_alpha_*: 0.0` 이라 스스로
  복구하지 않음). 경로 무효화보다 "틀린 위치로 계속 주행"이 더 큰 위험이다.
* 권장: 목표 실행 중에는 2D Pose Estimate 를 넣지 않는다. 필요하면 목표 취소 → 위치 지정 → AMCL 수렴 확인
  → 목표 재전송.

## 5. EKF: 원본 백업 vs 진단용 설정, 회전 중 비교

| | 원본 `ekf.yaml.bak.20261025_180237` | 2f8dbaf 의 `ekf.yaml`(10-25 진단용) |
|---|---|---|
| wheel `/wheel/odom` | vx, vy | vx, vy, **vyaw** |
| IMU `/imu/data` | **vyaw** | 제외 |

회전 명령 중 비교(클라우드, Gazebo 실측 자세 `/sim/ground_truth/odom` 기준, 정지 구간은 제외):

| 조건 | wheel/실제 | IMU/실제 | `/odom`/실제 | 결과 |
|---|---|---|---|---|
| 진단용 EKF + 기존 마찰 | 1.53 | 0.98 | **1.53** | Spin 은 "성공"하지만 실제로 덜 돎, AMCL 이 반복 보정(map→odom 점프 0.2~0.48 rad) |
| 원본 EKF + 기존 마찰 | 3.59 | 1.00 | **1.00** | 추정은 정확, 그러나 로봇이 \|ω\|≤0.34 rad/s 에서 거의 안 돌아 Spin 시간 초과 |
| 원본 EKF + 등방성 마찰(수정) | 1.53 | 1.01 | **1.00** | Spin 1.57/−3.14 성공, 앞 목표·뒤쪽(180°) 목표 성공 |

같은 조건에서 2f8dbaf 그대로(진단용 EKF, 기존 마찰) 뒤쪽 목표를 주면 휠 yaw 5.0 rad / IMU 2.2 rad 로 방향이
크게 틀려 남쪽으로 주행 → 장애물 정지 → Spin·BackUp 시간 초과 → `Failed to make progress` → 목표 실패가 재현됐다.

**결정**: EKF 는 원본 융합(휠 vx·vy + IMU vyaw)으로 되돌리고, 시뮬레이션 바퀴 마찰을 등방성(`mu2 1.0`)으로 바꿨다.
"정지 상태에서 각속도 0" 비교로는 이 차이가 보이지 않는다.

## 6. 조향 방식

실물 계획(인휠 4개 + 앞바퀴 조향 서보 2개)은 제자리 회전이 안 되지만, 시뮬레이션(스키드)과 Nav2 설정
(RPP rotate-to-heading, Spin 복구, NavFn)은 제자리 회전을 전제로 한다. 시뮬레이션 문제와 실물 과제 구분은
`docs/drive_kinematics_gap.md`.

## 7. 사용자 환경(Lyrical)에서 해야 할 검증 — 실행하지 않았음

```bash
cd ~/delivery_robot_ws && colcon build --symlink-install && source install/setup.bash
colcon test --packages-select robot_bringup robot_vision robot_safety && colcon test-result --verbose
```
예상: 새 시험(`test_single_camera_config`, `test_chain_diagnosis`, `test_person_hazard`,
`test_vision_hazard_input`) 통과. `robot_safety` 의 xmllint 는 schema 를 내려받지 못하면 실패할 수 있음(기존).

터미널 1:
```bash
ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=navigation motion_enabled:=true rviz:=true
```
터미널 2:
```bash
ros2 run robot_bringup motion_chain_monitor --ros-args -p use_sim_time:=true \
  -p ground_truth_topic:=/sim/ground_truth/odom -p output_file:=$HOME/chain_$(date +%H%M).jsonl
```
터미널 3 (확인 → 예상):
```bash
ros2 topic echo --once /camera/camera_info | grep -E "frame_id|width|height"   # camera_optical_frame, 640, 480
ros2 run tf2_ros tf2_echo base_link camera_optical_frame                        # 0.350 0 0.016, q(-0.5,0.5,-0.5,0.5)
ros2 topic echo --once /safety/state                                             # STOP:waiting_for_new_command
ros2 param set /sim_estop_publisher estop_active true ; sleep 2 ; ros2 topic echo --once /safety/state  # STOP:estop_active
ros2 param set /sim_estop_publisher estop_active false
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 3.14, time_allowance: {sec: 30}}"  # 성공, odom/truth ≈ 1.0
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
모니터를 Ctrl+C 하면 요약(정지 사유별 시간, 103/105 횟수, 회전 비교 `odom/truth`·`wheel/truth`)이 나온다.
기대값(클라우드 기준): `odom/truth` 0.95~1.05, `wheel/truth` 약 1.5. 다르게 나오면 그 수치를 보내 주면 된다.
