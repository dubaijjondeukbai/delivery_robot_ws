# robot_sim

배달로봇의 **Gazebo(gz-sim) 시뮬레이션**. `robot_description`의 URDF(sim:=true)를 전시장 월드에 띄우고,
`ros_gz_bridge`로 실제 로봇과 **같은 이름의 토픽**을 만듭니다. 그래서 `robot_vision` / `robot_navigation` / `robot_follower`는
한 줄도 바꾸지 않고 시뮬에서 그대로 돕니다.

```
worlds/exhibition.sdf                  12 x 8 m 전시장 (벽, 기둥 3, 부스 2, 선반, STATION_A QR 보드)
worlds/materials/textures/station_a_qr.png   QR 텍스처 ("STATION_A")
config/bridge.yaml                     gz ↔ ROS 토픽 매핑
launch/sim.launch.py                   robot_state_publisher + gz sim + spawn + bridge
```

## 토픽 (bridge.yaml)

| ROS | gz | 방향 | 비고 |
|---|---|---|---|
| `/scan` | `/scan` | gz→ROS | gpu_lidar 360°, 12 m, 10 Hz, frame `laser_link` |
| `/imu/data` | `/imu/data` | gz→ROS | 100 Hz, frame `imu_link` |
| `/camera/image_raw`, `/camera/camera_info` | 동일 | gz→ROS | 640×480 rgb8 15 Hz, frame `camera_optical_frame` (실제 intrinsics 포함 → AprilTag pose도 동작) |
| `/wheel/odom` | `/model/delivery_robot/odometry` | gz→ROS | DiffDrive 플러그인 odom (frame odom→base_link, TF는 bridge 안 함: EKF 담당). `_with_covariance` 토픽은 이 Gazebo 버전에서 발행되지 않아 사용 안 함 |
| `/joint_states` | `/joint_states` | gz→ROS | 바퀴 각도 |
| `/clock` | `/clock` | gz→ROS | 모든 노드 `use_sim_time:=true` |
| `cmd_vel_topic` (기본 `/cmd_vel_safe`) | `/model/delivery_robot/cmd_vel` | ROS→gz | 시뮬 구동 입력 = 실제 로봇의 robot_bridge 입력 |

## 실행

```bash
sudo apt install -y ros-lyrical-ros-gz          # gz sim + ros_gz_bridge (Lyrical 에 맞는 Gazebo 가 같이 설치됨)
cd ~/delivery_robot_ws && colcon build --symlink-install --packages-select robot_description robot_sim robot_bringup && source install/setup.bash

# 시뮬만
ros2 launch robot_sim sim.launch.py
ros2 launch robot_sim sim.launch.py gui:=false                 # headless (WSL2 GPU 없음) → RViz 로 확인
# 전체 (vision + follower + navigation, 모두 use_sim_time) — 보통은 이걸 씀
ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=mapping
```

지도 생성 → 저장 → navigation 순서는 `robot_navigation/README.md` 와 같고, 시뮬이므로 `motion_enabled:=true` 를 바로 써도 됩니다.
단 `robot_safety`가 아직 launch 에 없으면 `/cmd_vel_safe` 를 아무도 내지 않아 로봇이 움직이지 않습니다. 그때는:

```bash
ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=navigation motion_enabled:=true sim_cmd_vel_topic:=/cmd_vel_auto
```

(Nav2 → `/cmd_vel_auto` → 시뮬 직결. 시뮬 전용 우회이며 실제 로봇에서는 쓰지 않습니다.)

수동으로 밀어 보기:

```bash
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

## 확인

```bash
ros2 topic hz /scan /imu/data /wheel/odom /camera/image_raw
ros2 topic echo /wheel/odom --once          # frame_id odom, child_frame_id base_link
ros2 run tf2_tools view_frames              # base_link 아래 laser_link/imu_link/camera_*, 바퀴 4개
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

## 재시작 순서 / 알려진 증상

* **재시작 순서**: 1번 launch(Gazebo) → 토픽 확인 → RViz → teleop. Gazebo를 재시작하면 시뮬 시간이 0으로 돌아가 켜져 있던 RViz가 `Detected jump back in time` 뒤 TF 예외로 죽는다 → RViz도 다시 켠다.
* **launch 끄기 전에** 예전 Gazebo가 남아 있으면 `Found additional publishers on /clock` 경고와 함께 시계가 섞인다 → `pkill -f "gz sim"` 후 launch.
* DiffDrive 플러그인은 마지막 속도 명령을 유지한다(타임아웃 없음). teleop 은 `k`로 반드시 정지. 로봇을 원점으로 되돌리기:
  `gz service -s /world/exhibition/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "delivery_robot", position: {x: 0, y: 0, z: 0.3}'`

## WSL2 메모

* GUI는 WSLg 로 뜹니다. 검은 화면/느림이면 `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` 또는 `gui:=false` + RViz.
* gpu_lidar·camera 는 렌더링 기반이라 소프트웨어 렌더링에서는 real-time factor 가 0.3~0.6 정도로 떨어질 수 있습니다. 그래도 Nav2 테스트에는 충분합니다.
* 월드 그림자는 꺼 두었고 physics 는 250 Hz 입니다(`exhibition.sdf`).

## 바꿀 때

* 로봇 치수·센서 위치·모터 한계: `robot_description/urdf/robot_params.xacro`
* 센서 노이즈·마찰·플러그인: `robot_description/urdf/gazebo.xacro`
* 월드 배치: `worlds/exhibition.sdf` (실제 전시장 도면이 나오면 교체)
* 토픽 이름: `config/bridge.yaml` (`robot_navigation/config/localization.yaml` 과 맞출 것)
