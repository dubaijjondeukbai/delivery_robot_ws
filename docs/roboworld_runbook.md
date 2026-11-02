# 로보월드 전시 운영 순서 (2026-11, 실물 = 앞 2바퀴 조향, IMU 없음)

**전제** — 모두 갖춰져야 바퀴가 움직인다.
* Pi 4 Model B: Ubuntu 26.04 + Lyrical, 이 저장소 pull·빌드 (`rosdep ... --skip-keys "ros_gz_sim ros_gz_bridge"`)
* LiDAR 드라이버: `sllidar_ros2` (별도 워크스페이스), `frame_id:=laser_link`
* STM32 펌웨어가 `docs/stm32_drive_protocol.md` 의 DRV/FB/EST 를 구현 (QR 시연은 STOP → `DRIVE_STOPPED` 도)
* QR 시연의 뚜껑: ESP32 펌웨어가 `docs/esp32_lid_protocol.md` 의 `LID_*` 를 구현 (참고: `firmware/esp32_lid/`)
* **하드웨어 E-stop 이 모터 전원을 직접 끊음** (소프트웨어와 별개)
* `sudo apt install python3-serial ros-lyrical-teleop-twist-keyboard`
* 포트 이름은 `ls -l /dev/serial/by-id/` 로 확인해서 그 경로를 쓴다(재부팅하면 ttyUSB0/ttyACM0 순서가 바뀔 수 있음).
  아래 `/dev/ttyUSB0`, `/dev/ttyACM0` 은 예시.
* Pi 4 에서는 `robot.launch.py` 에 **`vision:=qr`** (카메라 + QR 만, YOLO·포즈 끔). 기본 `full` 은 Pi 4 에 무겁다.

공통 변수 (각 터미널에서):
```bash
source ~/delivery_robot_ws/install/setup.bash
EKF_WHEEL=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/ekf_wheel_only.yaml
```

## 1. 펌웨어 없이 Pi 쪽 확인 (가짜 STM32)
```bash
ros2 launch robot_base base.launch.py mock:=true          # T1
ros2 launch robot_bringup robot.launch.py                 # T2 (Safety + 비전)
ros2 topic echo /safety/state                             # T3: STOP:waiting_for_new_command 면 정상
```

## 2. 바퀴 띄우고 벤치 시험
```bash
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser_link   # T1
ros2 launch robot_base base.launch.py port:=/dev/ttyACM0                                           # T2
ros2 launch robot_bringup robot.launch.py                                                          # T3
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_auto        # T4
```
* 키: `i` 전진, `u`/`o` 전진하며 좌/우, `,` 후진은 Safety 가 막음.
  `j`/`l`(제자리 회전)은 ±60° 조향에서 **뒷차축 가운데를 축으로** 돈다(차체 중심이 약 0.23 m 원을 그림).
  키를 누르고 있어야 움직인다(0.5 s 명령 없으면 Safety 정지).
* 확인: 앞으로 → `/wheel/odom` linear.x > 0, 왼쪽(`u`) → angular.z > 0, 안쪽 앞바퀴가 더 꺾임.
  `[Base] link=OK fb=50Hz estop=clear` (T2 로그 5 초마다).

## 3. 지도 만들기 (부스 설치 후, 사람 없을 때)
```bash
ros2 launch robot_bringup robot.launch.py nav_mode:=mapping ekf_params_file:=$EKF_WHEEL   # T3 대신
# T4 teleop 으로 0.2 m/s 이하, 출발점으로 돌아와 한 바퀴 닫기, 뚜껑 닫고
ros2 run nav2_map_server map_saver_cli -f ~/maps/roboworld --ros-args -p map_subscribe_transient_local:=true
```

## 4. 시연
```bash
ros2 launch robot_bringup robot.launch.py nav_mode:=navigation motion_enabled:=true \
  map:=$HOME/maps/roboworld.yaml ekf_params_file:=$EKF_WHEEL                               # T3
```
1. 처음 위치: RViz `2D Pose Estimate` (또는 `set_initial_pose:=true initial_pose_x:=… y:=… yaw:=…`).
2. 지점 저장: 로봇을 시연 동선대로 손으로/teleop 으로 몰면서 지점마다
   `ros2 run robot_bringup stations record p1` … `p4`. **방향 = 지나가는 방향**으로 저장.
   (teleop 은 Nav2 와 같은 `/cmd_vel_auto` 를 쓰므로, 지점 저장할 때는 `nav_mode:=localization`
   으로 띄우거나 Nav2 목표가 없을 때만 조종한다.)
3. 순회: `ros2 run robot_bringup stations tour p1 p2 p3 p4 --loop --dwell 5`  (Ctrl+C = 목표 취소·정지)
4. 한 곳만: `ros2 run robot_bringup stations go p2`

### 동선 규칙 (앞바퀴 조향 ±60°, 후진 없음)
* 최소 회전반경(차체 중심) 약 0.6 m, Nav2 경로는 0.7 m 로 계획(`nav2.yaml minimum_turning_radius`).
  ±60° 는 실측 전 값이다(서보 링크 때문에 바퀴 각이 다를 수 있음) → 실측해서 `base.yaml max_steer_deg` 수정.
* **U턴은 폭 3 m 이상 빈 곳에서만**, 되도록 한 방향으로 도는 고리 동선(앞 0.8 m 정지 구역 때문).
* 벽·부스 모서리에서 **1 m 이상** 떨어진 동선. 앞 0.8 m(±35°) 안에 물체가 있으면 Safety 가 멈추는데,
  벽을 마주 보고 멈추면 후진이 안 돼 **스스로 못 빠져나온다** → E-stop 후 손으로 옮긴다.
* 사람이 앞 0.8 m 안에 들어오면 멈췄다가, 비키면 다시 간다(정상).
* 시뮬레이션 확인(클라우드, Jazzy): 직진·완만한 곡선 목표는 성공. 장애물 가까운 U턴 목표는 0.8 m 정지
  구역에 걸려 막혔다(±30° 일 때 시험) → 위 규칙이 필요한 이유. ±60° 로 U턴은 다시 시험하지 않았다.

## 4-1. QR 배달 시연 (출발 QR → 이동 → 정지 → 도착 QR → 뚜껑 열기)

QR 내용(문자열 그대로 QR 코드로 만든다): `START:A` (A 로 출발), `OPEN:A` (A 에서 뚜껑 열기), `CLOSE` (닫기).
지점 이름 = QR 의 A/B (영문·숫자·`_`·`-`, 40자 이내).

```bash
# 준비: 4 의 T3 대신 (Pi 4 이므로 vision:=qr)
ros2 launch robot_bringup robot.launch.py nav_mode:=navigation motion_enabled:=true vision:=qr \
  map:=$HOME/maps/roboworld.yaml ekf_params_file:=$EKF_WHEEL                               # T3
# 지점 저장 (처음 한 번): 로봇을 A 위치·도착 방향으로 놓고
ros2 run robot_bringup stations record A        # B 도 같은 방법. ~/maps/stations.yaml 에 저장
# 시연 노드 + ESP32 브리지 (팀원 esp32_serial_bridge 를 함께 띄움)
ros2 launch robot_bringup delivery_demo.launch.py esp32_port:=/dev/serial/by-id/<ESP32>       # T5
ros2 topic echo /delivery/state                                                                # T6
```

흐름과 규칙 (`/delivery/state` 의 `state`):
1. `IDLE` 에서 카메라에 `START:A` → `NAVIGATING`. 이동 중에는 `OPEN`·다른 `START` 를 무시한다.
2. Nav2 도착 → STM32 에 STOP → `/odom` 이 0.5 s 동안 0 **이고** STM32 `DRIVE_STOPPED` → `WAIT_OPEN`.
   5 s 안에 확인이 안 되면 `ERROR`(뚜껑 안 엶). STM32 펌웨어가 `DRIVE_STOPPED` 를 아직 못 보내면
   T5 에 `require_drive_stopped:=false` 를 붙인다(`/odom` 정지만 확인).
3. `OPEN:A` (도착지와 같을 때만) → ESP32 `LID_OPEN`. 리미트 스위치가 없으므로 `OPEN_UNCONFIRMED`
   (열림, 센서 확인 안 됨)로 표시된다 → **눈으로 확인**. 도착지가 다른 `OPEN:B` 는 열지 않는다.
4. 물건을 꺼낸 뒤 `CLOSE` QR (또는 아래 운영자 명령) → `IDLE` (뚜껑 `CLOSED_UNCONFIRMED`). 다음 `START` 가능.
   같은 `START:A` 를 계속 보여 줘도 끝난 뒤 3 s 동안은 다시 출발하지 않는다.

운영자 명령 (QR 없이, 노트북/Pi 터미널에서):
```bash
ros2 topic pub --once /delivery/command std_msgs/msg/String "{data: START:A}"   # OPEN:A, CLOSE 도 같음
ros2 topic pub --once /delivery/command std_msgs/msg/String "{data: CANCEL}"    # 이동 취소(그 자리 정지)
ros2 topic pub --once /delivery/command std_msgs/msg/String "{data: RESET}"     # ERROR 해제
```
* `ERROR` 가 나면: 원인 해결 → 뚜껑이 열려 있거나 상태가 `UNKNOWN` 이면 `CLOSE` → `RESET`.
  RESET 은 뚜껑을 닫혔다고 가정하지 않는다(뚜껑이 닫힘이 아니면 START 를 거부하고 CLOSE 를 안내).
* ESP32 를 켜기 전에 뚜껑을 손으로 닫아 둔다(스위치가 없어 켜질 때 닫혀 있다고 본다).
* 뚜껑이 열린 채로는 수동 조종(teleop)을 하지 않는다(LiDAR 뒤쪽이 가려짐, 시연 노드는 막지 못함).

펌웨어 없이 먼저 시험 (가짜 뚜껑, 시뮬):
```bash
ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=navigation motion_enabled:=true vision:=qr
ros2 launch robot_bringup delivery_demo.launch.py sim:=true mock_lid:=true
ros2 topic pub --once /vision/qr_result std_msgs/msg/String "{data: START:A}"
```

## 5. 문제가 생기면
| 증상 | 확인 |
|---|---|
| `STOP:waiting_for_estop_state` | base.launch.py 실행 중? STM32 가 `EST,0` 을 보내나? |
| `[Base] link=DOWN` | 포트(`ls /dev/ttyACM*`), 펌웨어 FB, 다른 프로세스가 포트를 잡았나(팀원 stm32_serial_bridge) |
| `[Base] 명령 조정` | 회전반경 0.01~0.54 m(뒷차축 기준) 명령은 조향 한계 때문에 가장 가까운 가능한 명령으로 바뀜 — 정상 |
| 목표가 안 끝남, 원을 그림 | 동선이 너무 급함 → 지점 간격·방향 조정 |
| 벽 앞에서 `STOP:obstacle_detected` 계속 | 후진 불가 → E-stop, 손으로 옮기고 `stations` 다시 |
| `[Delivery] 정지 확인 실패: ... STM32 마지막 상태=받은 적 없음` | STM32 가 `DRIVE_STOPPED` 를 안 보냄 → 펌웨어 확인, 급하면 `require_drive_stopped:=false` |
| `뚜껑이 닫혀 있지 않아 출발하지 않음` | `CLOSE` 로 닫기(복구용, IDLE 에서도 됨) 후 다시 `START` |
| `등록되지 않은 지점: A` | `stations record A` 안 함 / `~/maps/stations.yaml` 확인 (START 때마다 다시 읽음) |
| 뚜껑이 안 움직이고 시간 초과 | ESP32 포트(`esp32_port`), 펌웨어가 `LID_*` 를 아는지, 시리얼 터미널로 `LID_OPEN` 직접 시험 |

기록: `ros2 run robot_bringup motion_chain_monitor --ros-args -p output_file:=$HOME/run_$(date +%H%M).jsonl`
