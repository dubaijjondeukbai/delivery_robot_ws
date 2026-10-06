# robot_navigation

배달로봇의 **localization / navigation 패키지**. 커스텀 노드 없이 표준 패키지
(`robot_localization`, `slam_toolbox`, `nav2_map_server`, `nav2_amcl`, Nav2 서버들)를
launch + YAML 로 연결한다. Nav2 의 속도 출력은 **반드시** `/cmd_vel_auto` 로 나가 기존
`robot_safety → robot_control → robot_bridge → STM32` 경로를 그대로 통과한다.

```
                 LiDAR ──/scan──────────────────────────────┐
  Wheel Encoder ──/wheel/odom──┐                            │
  IMU ───────────/imu/data─────┴─► ekf_filter_node ──/odom──┤  (+ TF odom -> base_link)
                                                            │
      MODE A  slam_toolbox  ◄────────────────────────────────┤──► /map, TF map->odom  ──► map_saver_cli ──► map.yaml/.pgm
                                                            │
      MODE B  map_server ──/map──► amcl  ◄───────────────────┘──► TF map->odom, /amcl_pose
                                     │
                                     ▼
                /goal_pose ──► Nav2 (planner / controller / behaviors / bt_navigator / velocity_smoother)
                                     │ cmd_vel_smoothed  (remap)
                                     ▼
                   motion_enabled:=true  → /cmd_vel_auto → robot_safety → /cmd_vel_safe → robot_control → robot_bridge → STM32
                   motion_enabled:=false → /cmd_vel_nav_dry_run (아무도 구독하지 않음, 기본값)
```

TF 트리: `map → odom → base_link → laser_link / imu_link`
* `map → odom` : AMCL(운행) / slam_toolbox(지도 생성)
* `odom → base_link` : `ekf_filter_node` **만** (robot_bridge 는 `/wheel/odom` 메시지만 publish, TF 금지)
* `base_link → 센서` : `robot_bringup` 의 URDF 가 생기기 전까지 이 패키지의 `publish_sensor_tf`(기본 true)

## 패키지 구조 / 선택 이유

```
robot_navigation/
├── config/
│   ├── localization.yaml   # "wiring": 토픽 이름, 센서 장착 TF, launch 기본값 (launch 가 읽음)
│   ├── ekf.yaml            # robot_localization ekf_node
│   ├── slam.yaml           # slam_toolbox (mapping)
│   ├── amcl.yaml           # map_server + amcl
│   └── nav2.yaml           # Nav2 서버 전체 (Lyrical / Nav2 1.5 파라미터 이름)
├── launch/
│   ├── mapping.launch.py       # MODE A: static TF + EKF + slam_toolbox
│   ├── localization.launch.py  # MODE B: static TF + EKF + map_server + AMCL
│   └── navigation.launch.py    # MODE B 포함 + Nav2 (+ cmd_vel 라우팅)
├── robot_navigation/
│   ├── __init__.py
│   └── launch_common.py    # launch 공용 함수 (wiring 읽기, remap, EKF/TF 노드) — ROS 노드 아님
├── maps/README.md          # 지도 저장/배치 방법. 저장한 map.yaml/.pgm 을 여기에 둔다
├── package.xml / setup.py / setup.cfg / README.md   (.gitignore 는 workspace 루트에 하나만 둔다)
```

* **이름 `robot_navigation`**: `robot_localization_stack` 은 실제 ROS 패키지 `robot_localization` 과 혼동되고,
  이 패키지는 localization 뿐 아니라 Nav2 까지 담으므로 `robot_navigation` 이 자연스럽다.
* **`ament_python`**: 워크스페이스의 다른 패키지와 같은 빌드 타입이고, launch 파일들이 공유하는 작은 Python
  모듈(`launch_common.py`)을 설치하기에 편하다(`nav2_bringup` 이 `nav2_common` 을 쓰는 것과 같은 패턴).
  launch/config 만 있는 패키지는 `ament_cmake` 로도 가능하지만 CMakeLists.txt 만 늘어난다. 실행 파일(console_scripts)은 없다.
* **커스텀 노드 없음**: SLAM/AMCL 재구현은 당연히 하지 않았고, watchdog 류의 노드도 넣지 않았다. 아래 "Safety" 에서
  `robot_safety` 가 무엇을 봐야 하는지 명시했다.

## 1. 토픽 / 프레임 가정과 변경 방법

| 역할 | 기본 토픽 | 타입 | 변경 위치 |
|---|---|---|---|
| LiDAR | `/scan` | `sensor_msgs/LaserScan` | `localization.yaml` `topics.scan` 또는 `scan_topic:=` |
| Wheel odom (raw) | `/wheel/odom` | `nav_msgs/Odometry` | `topics.wheel_odom` / `wheel_odom_topic:=` |
| IMU | `/imu/data` | `sensor_msgs/Imu` | `topics.imu` / `imu_topic:=` |
| EKF 출력 | `/odom` | `nav_msgs/Odometry` | `topics.odom` / `odom_topic:=` |
| Map | `/map` | `nav_msgs/OccupancyGrid` | `topics.map` / `map_topic:=` |
| Nav goal | `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 기본 |
| Nav2 출력 | `/cmd_vel_auto` | `geometry_msgs/Twist` | `topics.cmd_vel_out` / `cmd_vel_out_topic:=` |

구현 방식: 네 개의 노드 YAML 은 **상대 이름**(`scan`, `odom`, `map`, `wheel/odom`, `imu/data`)만 쓰고,
launch 가 `localization.yaml` 의 값으로 remap 한다. 그래서 토픽 이름은 한 곳에서만 바꾼다.
프레임 이름(REP-105: `map/odom/base_link/laser_link/imu_link`)은 remap 대상이 아니므로 네 YAML 에 각각 적혀 있다.

**입력 데이터 요구사항 (robot_bridge / 드라이버가 지켜야 함)**

* `/wheel/odom`: `header.frame_id: odom`, `child_frame_id: base_link`, `twist.covariance` 대각 성분 > 0
  (0 이면 "완벽한 센서"로, -1 이면 "모름"으로 취급되어 **융합되지 않음**). TF 는 publish 하지 않는다.
* `/imu/data`: `header.frame_id: imu_link`, REP-103 축(x 전방, y 좌, z 상), yaw 반시계 = +. `angular_velocity_covariance` 대각 > 0.
* `/scan`: `header.frame_id: laser_link`, `range_min/range_max` 정확히.

## 2. EKF (robot_localization) — 무엇을 융합하는가

`odom0_config` / `imu0_config` 는 15 개 boolean 으로, 순서는 고정이다.

```
[ x  y  z | roll pitch yaw | vx vy vz | vroll vpitch vyaw | ax ay az ]
```

| 입력 | true | 이유 |
|---|---|---|
| wheel odom | `vx, vy, vyaw` | pose(x,y,yaw)는 같은 엔코더 속도의 적분값이라 둘 다 넣으면 **같은 정보를 두 번** 융합하게 된다. 속도만 넣으면 EKF 가 스스로 적분한다. `vy=0` 은 "옆으로 못 미끄러진다"는 유효한 제약. |
| IMU | `vyaw` | 자이로가 가장 좋은 yaw-rate 센서. yaw 각도는 6축 IMU 라 드리프트해서 제외, 가속도는 bias 가 속도 드리프트가 되어 제외(검증 후 `ax` 만 고려). |

`two_d_mode: true`(z/roll/pitch 고정), `world_frame: odom`(이 노드는 `odom→base_link` 만 담당, `map→odom` 은 AMCL).
휠 슬립이 심하면 wheel odom 의 `vyaw` 를 false 로 바꿔 자이로에 맡긴다. 자세한 설명은 `config/ekf.yaml` 주석.

## 3. AMCL vs slam_toolbox localization mode

| | **Nav2 AMCL (기본값으로 채택)** | slam_toolbox `mode: localization` |
|---|---|---|
| 원리 | 입자 필터 + likelihood field | 포즈 그래프에 새 스캔을 임시로 붙이는 스캔 매칭 |
| 입력 지도 | `map.yaml/.pgm` (map_server) | `.posegraph/.data` (serialize_map 결과) |
| 정확도 | 양호 | 특징이 많은 환경에서 더 정밀 |
| 환경 변화 | 집기 이동에 어느 정도 강인(likelihood field) | 변화를 지도에 임시 반영해 더 강인 |
| CPU (Pi 5) | 가벼움(500~2000 입자, 60 beams) | 더 무거움 |
| 초기 위치/복구 | `/initialpose`, 전역 재초기화 서비스, `/amcl_pose` **공분산 제공** | 초기 위치 처리 약함, 공분산 없음 |
| 안전 측면 | 공분산·입자 분포로 "자신감"을 상위에서 감시 가능 | 실패를 상위에서 판단할 신호가 적음 |

30 kg 로봇의 안전 설계에서는 "지금 위치 추정을 얼마나 믿을 수 있는가"를 `robot_safety` 가 읽을 수 있어야 하므로
**AMCL 을 기본값**으로 한다. 집기 배치가 자주 바뀌어 AMCL 정합이 떨어지면 그때 `slam.yaml` 을 복사해
`mode: localization`, `map_file_name: <posegraph 경로>`, `map_start_at_dock: true` 로 둔 설정으로 `localization_slam_toolbox_node` 를 띄우는 대안을 검토한다.

AMCL 초기값 근거(요약, 상세는 `config/amcl.yaml`): 입자 500~2000(기본 대역, Pi 5 여유), `update_min_d/a` 0.1(느린 로봇이 보정 사이에 25 cm 밀리지 않도록 기본보다 촘촘히),
`transform_tolerance` 1.0(Pi 스케줄링 지터에 TF 끊김 방지), `recovery_alpha_*` 0(무작위 입자 주입으로 **갑자기 점프하는 것**보다 "lost" 상태가 보이는 편이 안전),
`alpha1~5` 0.2(구동계 미확정이므로 odometry 를 보수적으로 신뢰), `laser_max_range` 12 m(A2급 LiDAR), `max_beams` 60.

## 4. cmd_vel 설계와 Safety

Nav2 내부 흐름(Humble 이후 nav2_bringup 과 동일): `controller_server`/`behavior_server` → `cmd_vel`(remap `cmd_vel_nav`) → `velocity_smoother` → `cmd_vel_smoothed`.
`cmd_vel_smoothed` 를 launch 에서 remap 하므로 Nav2 의 **단일 출구**는 velocity_smoother 하나다. collision_monitor 는 띄우지 않는다(LiDAR 정지는 `robot_safety` 담당).

| 요구사항 | 구현 |
|---|---|
| 1,2. localization loss / 낮은 confidence 가 모터 명령으로 직결되지 않음 | AMCL 은 TF 와 `/amcl_pose` 만 publish. recovery 입자 주입 OFF. Nav2 가 TF 를 못 찾으면 controller 가 실패 → 명령 publish 중단. |
| 3. scan/odom/localization/TF 실패 시 상위에서 정지 | `velocity_smoother.velocity_timeout: 1.0`(1 s 무명령 → 0 publish), `controller_server.publish_zero_velocity: true`. **`robot_safety` 가 추가로 봐야 할 것**: `/scan` 수신 간격, `/odom` 수신 간격, `/amcl_pose` 의 age 와 `covariance[0]`,`[7]`,`[35]`(예: 위치 분산 > 0.5 m² 이면 정지), `map→base_link` TF 의 age(`transform_tolerance` 1 s 초과), `/cmd_vel_auto` 자체의 timeout. |
| 4,5. robot_safety 우회 금지, Nav2→STM32 직결 금지 | Nav2 출력은 `/cmd_vel_auto` 로만 remap. robot_bridge 는 `/cmd_vel_safe` 계열만 구독. |
| 6. 센서 단절 fail-safe | EKF `sensor_timeout` 0.2 s(해당 입력 없이 예측, diagnostics 경고), AMCL/costmap 은 `/scan` 없으면 갱신 중단 → RPP `use_collision_detection` 가 costmap 기준으로 정지, 최종 정지는 robot_safety. |
| 7. 벤치 테스트 전 실제 motion 금지 | `motion_enabled` 는 **기본 false, 파일에서 읽지 않음**. false 면 Nav2 출력이 `/cmd_vel_nav_dry_run`(dead end)으로 가서 planning·localization 만 관찰 가능. `motion_enabled:=true` 를 명령줄에 명시해야만 `/cmd_vel_auto` 로 나간다. |

속도 제한(모두 `TODO: tune on real robot`): 직진 0.3 m/s, 회전 0.5~0.6 rad/s, 후진 -0.1 m/s(BackUp recovery 용, 금지하려면 `min_velocity: [0.0, ...]`), 가속 0.3 m/s², 감속 0.5 m/s².
`enable_stamped_cmd_vel: false` 를 controller/behavior/velocity_smoother 에 명시했다 — Kilted 부터 Nav2 기본 출력이 `TwistStamped` 로 바뀌었기 때문에, 이것이 없으면 robot_safety(`Twist`)가 메시지를 받지 못한다.

## 5. 실행 순서도

```
[MAP CREATION — MODE A]                    [ROBOT OPERATION — MODE B]
LiDAR /scan                                Saved map.yaml/.pgm ─► map_server ─► /map
Wheel /wheel/odom ─┐                       LiDAR /scan ──────────────────────────┐
IMU   /imu/data  ──┴─► EKF ─► /odom        Wheel+IMU ─► EKF ─► /odom (odom→base_link)
                        │                                   │
                        ▼                                   ▼
                  slam_toolbox ─► /map, map→odom          AMCL ─► map→odom, /amcl_pose
                        │                                   │
                        ▼                                   ▼
          map_saver_cli ─► ~/maps/*.yaml/*.pgm       Nav2 (planner → controller → velocity_smoother)
          serialize_map ─► ~/maps/*.posegraph               │ /cmd_vel_auto (motion_enabled:=true)
                                                            ▼
                                              robot_safety ─► robot_control ─► robot_bridge ─► STM32
```

## 6. 설치 / 빌드 / 실행 명령

```bash
# --- 의존성 (Ubuntu 26.04 + ROS 2 Lyrical 기준; 다른 distro 는 ${ROS_DISTRO} 가 알아서 바뀜)
source /opt/ros/lyrical/setup.bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-rviz2
# 또는 package.xml 기준 자동 설치
cd ~/delivery_robot_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

# --- 빌드
colcon build --packages-select robot_navigation --symlink-install
source install/setup.bash
ros2 launch robot_navigation navigation.launch.py --show-args      # 인자 확인

# --- 지도 생성 (MODE A). 다른 터미널에서 teleop/follower 로 로봇을 천천히 몰고 다닌다
ros2 launch robot_navigation mapping.launch.py

# --- 지도 저장 (mapping 실행 중인 상태에서)
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/exhibition_map --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/exhibition_map'}"
# (선택) 패키지에 포함시켜 map:= 없이 쓰기
cp ~/maps/exhibition_map.yaml ~/maps/exhibition_map.pgm ~/delivery_robot_ws/src/robot_navigation/maps/ && colcon build --packages-select robot_navigation --symlink-install

# --- Localization 만 (MODE B)
ros2 launch robot_navigation localization.launch.py map:=$HOME/maps/exhibition_map.yaml

# --- Nav2 navigation: 먼저 dry-run (기본, 모터로 아무것도 가지 않음)
ros2 launch robot_navigation navigation.launch.py map:=$HOME/maps/exhibition_map.yaml
# 벤치 테스트 후 실제 주행 (robot_safety 가 켜져 있는 상태에서만!)
ros2 launch robot_navigation navigation.launch.py map:=$HOME/maps/exhibition_map.yaml motion_enabled:=true

# --- 자주 쓰는 override
ros2 launch robot_navigation mapping.launch.py scan_topic:=/lidar/scan wheel_odom_topic:=/stm32/odom publish_sensor_tf:=false
```

**초기 위치**: AMCL 은 시작 위치를 알아야 한다. RViz 의 "2D Pose Estimate" 로 찍거나, 터미널에서

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```

로봇이 항상 같은 도킹 위치에서 켜진다면 `amcl.yaml` 의 `set_initial_pose: true` + `initial_pose` 를 쓴다.

## 7. 디버깅

```bash
ros2 node list                                   # ekf_filter_node, map_server, amcl, lifecycle_manager_*, Nav2 서버들
ros2 lifecycle get /amcl                         # active 여야 함 (unconfigured 면 map 경로/파라미터 오류 확인)
ros2 topic list
ros2 topic hz /scan                              # LiDAR ~10 Hz
ros2 topic hz /wheel/odom ; ros2 topic hz /imu/data
ros2 topic echo /odom --once                     # EKF 출력. frame_id odom / child base_link
ros2 topic echo /amcl_pose                       # pose + covariance (localization 신뢰도)
ros2 run tf2_ros tf2_echo map base_link          # 전체 TF 체인. "Could not transform" 이면 어느 구간이 빠졌는지 아래로 확인
ros2 run tf2_ros tf2_echo odom base_link         # EKF 가 publish
ros2 run tf2_ros tf2_echo base_link laser_link   # 정적 TF
ros2 run tf2_tools view_frames                   # frames.pdf 생성: 부모가 2개인 프레임(TF 충돌)이 있는지 확인
ros2 topic echo /cmd_vel_nav_dry_run             # dry-run 중 Nav2 가 내려는 속도
ros2 topic echo /cmd_vel_auto                    # motion_enabled:=true 일 때만 데이터가 보여야 함
ros2 topic echo /diagnostics                     # EKF 가 거부하는 입력(covariance 0/-1 등)
```

RViz (랩톱에서 `ROS_DOMAIN_ID` 를 Pi 와 맞춰 실행):

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

확인 항목
1. Fixed Frame = `map`. TF 트리에 `map → odom → base_link → laser_link` 가 모두 보이고 끊김 없음.
2. `/map` 이 표시되고, `/scan` 점들이 지도의 벽과 겹침(안 겹치면 초기 위치가 틀렸거나 `laser_link` TF yaw 가 틀림).
3. `/particlecloud`(AMCL 입자)가 주행하며 수렴하는지. 넓게 퍼지면 lost.
4. mapping 중에는 지도가 정상적으로 자라고 루프를 돌아왔을 때 벽이 이중으로 생기지 않는지(loop closure).
5. navigation 에서 `/plan`(전역 경로), local/global costmap, footprint(`robot_radius`)가 로봇 크기와 맞는지.
6. Nav2 Goal 을 찍은 뒤 dry-run 이면 `/cmd_vel_nav_dry_run` 에만 속도가 나오는지, `/cmd_vel_auto` 는 침묵하는지.

## 8. 주의 / 알려진 제약

* `nav2.yaml` 은 **Nav2 1.5 (Lyrical)** 이름을 쓴다: RPP `max_linear_vel`(구 `desired_linear_vel`), controller `PathHandler`, `action_server_result_timeout` 제거.
  Kilted 이하에서 쓰려면 주석의 구 이름으로 바꾼다. 알 수 없는 파라미터는 Nav2 가 무시하므로 치명적이진 않지만 속도 제한이 기본값으로 돌아갈 수 있으니 `ros2 param get /controller_server FollowPath.max_linear_vel` 로 확인.
* `robot_radius` 0.35 m, 센서 장착 오프셋, LiDAR 사거리, 속도 제한은 전부 placeholder(`TODO: tune on real robot`).
* `robot_bringup` 이 URDF/robot_state_publisher 를 publish 하기 시작하면 `publish_sensor_tf:=false`(또는 `localization.yaml` `sensor_tf.publish: false`). 두 곳에서 같은 TF 를 내면 지터와 경고가 발생한다.
* Pi 5 에서 lifecycle 로그에 "bond ... timed out" 이 보이면 `launch_common.lifecycle_manager` 의 `bond_timeout` 을 10.0 으로 올린다.
* slam_toolbox 는 nav2 lifecycle manager 로 올리면 `bad_weak_ptr` 로 activate 가 실패하는 릴리스가 있어(Lyrical 에서 확인) mapping.launch.py 가 configure/activate 이벤트를 직접 보낸다(`use_lifecycle_manager: false`).
* `robot_vision` 과는 의존성이 없다. QR 결과를 목적지로 쓰는 것은 상위(task_manager)에서 `/goal_pose` 또는 NavigateToPose 액션으로 변환하면 된다. AprilTag 는 station docking 전용으로 남긴다.
