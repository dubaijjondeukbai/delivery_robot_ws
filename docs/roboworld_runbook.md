# 로보월드 전시 운영 순서 (2026-11, 실물 = 앞 2바퀴 조향, IMU 없음)

**전제** — 모두 갖춰져야 바퀴가 움직인다.
* Pi: Ubuntu 26.04 + Lyrical, 이 저장소 pull·빌드 (`rosdep ... --skip-keys "ros_gz_sim ros_gz_bridge"`)
* LiDAR 드라이버: `sllidar_ros2` (별도 워크스페이스), `frame_id:=laser_link`
* STM32 펌웨어가 `docs/stm32_drive_protocol.md` 의 DRV/FB/EST 를 구현
* **하드웨어 E-stop 이 모터 전원을 직접 끊음** (소프트웨어와 별개)
* `sudo apt install python3-serial ros-lyrical-teleop-twist-keyboard`

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
* 키: `i` 전진, `u`/`o` 전진하며 좌/우 (앞바퀴 조향이라 `j`/`l` 제자리 회전은 거부됨, `,` 후진은 Safety 가 막음).
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

### 동선 규칙 (앞바퀴 조향 ±30°, 후진 없음)
* 최소 회전반경 약 1.1 m. **U턴은 폭 3 m 이상 빈 곳에서만**, 되도록 한 방향으로 도는 고리 동선.
* 벽·부스 모서리에서 **1 m 이상** 떨어진 동선. 앞 0.8 m(±35°) 안에 물체가 있으면 Safety 가 멈추는데,
  벽을 마주 보고 멈추면 후진·제자리 회전이 안 돼 **스스로 못 빠져나온다** → E-stop 후 손으로 옮긴다.
* 사람이 앞 0.8 m 안에 들어오면 멈췄다가, 비키면 다시 간다(정상).
* 시뮬레이션 확인(클라우드, Jazzy): 직진·완만한 곡선 목표는 성공. 장애물 가까운 U턴 목표는 0.8 m 정지
  구역에 걸려 막혔다 → 위 규칙이 필요한 이유.

## 5. 문제가 생기면
| 증상 | 확인 |
|---|---|
| `STOP:waiting_for_estop_state` | base.launch.py 실행 중? STM32 가 `EST,0` 을 보내나? |
| `[Base] link=DOWN` | 포트(`ls /dev/ttyACM*`), 펌웨어 FB, 다른 프로세스가 포트를 잡았나(팀원 stm32_serial_bridge) |
| `[Base] 명령 조정(infeasible)` | 제자리 회전 명령(teleop `j`/`l`) — 정상 거부 |
| 목표가 안 끝남, 원을 그림 | 동선이 너무 급함 → 지점 간격·방향 조정 |
| 벽 앞에서 `STOP:obstacle_detected` 계속 | 후진 불가 → E-stop, 손으로 옮기고 `stations` 다시 |

기록: `ros2 run robot_bringup motion_chain_monitor --ros-args -p output_file:=$HOME/run_$(date +%H%M).jsonl`
