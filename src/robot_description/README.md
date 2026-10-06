# robot_description

배달로봇의 **URDF(xacro) 모델**과 `robot_state_publisher` launch. 치수는 Fusion OBJ(`전체_배치.obj`, 2026-10-05)에서 읽었고,
센서 위치는 디자인 스케치 기준 추정값입니다(모두 `urdf/robot_params.xacro` 한 파일에 있음).

```
urdf/robot_params.xacro   ← 모든 숫자(치수·질량·센서 위치·모터 한계). 수정은 여기서만
urdf/macros.xacro         ← 관성/바퀴 매크로
urdf/robot.urdf.xacro     ← 모델 본체 (sim:=true 면 gazebo.xacro 포함)
urdf/gazebo.xacro         ← Gazebo 센서(gpu_lidar·imu·camera)·DiffDrive(4륜)·JointStatePublisher
launch/description.launch.py
```

## TF

```
base_footprint (바닥)
base_link (로봇 중심, 바퀴축 높이 z=0.0857) ─┬─ laser_link            (스캔면, 바닥에서 0.70 m)
                                           ├─ imu_link              (섀시 중심, 0.10 m)
                                           ├─ camera_link ─ camera_optical_frame (0.45 m, 전방)
                                           └─ wheel_{front,rear}_{left,right}_link
```

`map→odom`(AMCL/slam_toolbox), `odom→base_link`(EKF)는 `robot_navigation`이 publish 합니다.
이 패키지가 `base_link→센서` TF를 내므로 `robot_navigation`은 `publish_sensor_tf:=false` 로 실행합니다(`robot_bringup`이 그렇게 넘김).

## 확인

```bash
ros2 launch robot_description description.launch.py publish_joint_states:=true
ros2 run rviz2 rviz2            # Fixed Frame = base_link, RobotModel(topic /robot_description), TF 추가
xacro $(ros2 pkg prefix robot_description)/share/robot_description/urdf/robot.urdf.xacro sim:=true > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
```

## 수치 중 실측이 필요한 것 (robot_params.xacro 의 TODO)

| 항목 | 현재값 | 비고 |
|---|---|---|
| `wheel_max_rpm` | 150 | NURI WA172E 24 V 데이터시트 정격 rpm → 최고 속도 자동 계산 |
| `wheel_mass` / `body_mass` | 2.5 / 25 kg | 합계 35 kg 가정 |
| `lidar_z` | 0.70 m | 핸들 아치 위 스캔면 높이 |
| `camera_*` | (0.26, 0, 0.45), 수평 | 적재함 앞면 중앙 |
| `imu_*` | 섀시 중심 | |
| `wheel_mu2` | 0.5 | 스키드 선회 마찰 (시뮬) |
