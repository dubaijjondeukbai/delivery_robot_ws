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

## 2026-10-27 치수 갱신 (CompleteDesign.zip)

Fusion 전체 설계 OBJ(`Complete+Design.obj`, mm, Z-up)의 바운딩 박스로 `urdf/robot_params.xacro` 를 갱신했다.
메시 축 → ROS 축은 메시 -Y = 전방(x), 메시 +X = 좌측(y). 전방 = 아치·LiDAR 쪽 끝(사용자 확인).

| 항목 | 이전 | CAD |
|---|---|---|
| 축거 / 윤거 | 0.400 / 0.550 | 0.460 / 0.541 |
| 바퀴 반지름 / 폭 | 0.0857 / 0.052 | 0.0857 / 0.0545 |
| 하부 섀시 | 0.60 × 0.30 × 0.10 | 0.75 × 0.431 × 0.125 |
| 적재함 | 0.502 × 0.570 × 0.380 (바닥 0.22) | 0.781 × 0.642 × 0.500 (바닥 0.15) + 측면 디스플레이 → 전체 폭 0.708 |
| 아치 | 중앙 | 전방 끝 x 0.233, z 0.525~0.74 |
| LiDAR 스캔면 | (0, 0, 0.700) | (0.261, 0, 0.783 추정) — 디테일 모델 기준 |
| 카메라 | 1개 (0.26, 0, 0.45) | 전방 중앙 1개, 전면 하단 범퍼 (0.350, 0, 0.102) 추정(TODO 실측). 2대(전방 좌우) 구성은 `robot_params.xacro` 주석 절차로만 전환하며 좌우 간격·높이는 실측 전이라 값을 넣지 않음(2026-11-01, `front_camera_count` 제거). 얼굴 '눈' 부품(z 0.40)은 카메라가 아님 |
| 외곽 footprint | ±0.30 × ±0.30 | ±0.395 × ±0.354 (카메라 렌즈 끝 포함) |
| 뚜껑(열림) | — | LiDAR 뒤 0.05 m, z 0.63~1.14, 후방 약 157° 가림 (정지 중에만 열림 전제) |

추정값(실측 필요): LiDAR 스캔면 높이(외함 바닥 0.750 + 0.030), 카메라 pitch, IMU 위치, 질량/관성.
같이 바꾼 파일: `robot_navigation/config/nav2.yaml`(footprint ±0.391 × ±0.354), `robot_navigation/config/localization.yaml`(예비 laser TF),
`robot_safety/config/safety.yaml`(주석: LiDAR 원점이 전면에서 0.138 m 뒤).
Gazebo 는 전방 중앙 카메라 1대만 시뮬레이션한다. 2026-11-01 바퀴 마찰을 등방성(`wheel_mu2` 1.0)으로 바꿨고,
진단용 지면 실측 `/sim/ground_truth/odom`(Gazebo OdometryPublisher, Nav2/EKF 는 사용하지 않음)을 추가했다.
