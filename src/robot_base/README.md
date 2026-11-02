# robot_base — 조향 베이스 연결 (실물 전용)

실물: 인휠 모터 4개 + **앞 2바퀴 조향**(서보 2개, ±30° 가정), IMU 없음.

Nav2/robot_safety 의 속도 명령을 앞바퀴 조향각·바퀴 4개 속도로 바꿔 STM32 로 보내고, STM32 가 돌려준
바퀴 상태로 `/wheel/odom` 과 `/estop_state` 를 만든다. 시뮬레이션(Gazebo)에서는 쓰지 않는다.

```
/cmd_vel_safe (Twist) ─► base_bridge_node ─ DRV 줄 ─► STM32 ─► 앞바퀴 조향 서보 2 + 인휠 모터 4
                              ▲   │
          FB / EST / IMU 줄 ──┘   ├─► /wheel/odom (Odometry, TF 없음 → EKF)
                                  ├─► /estop_state (Bool 10 Hz, 끊기면 true → robot_safety 정지)
                                  ├─► /imu/data (STM32 가 IMU 줄을 보낼 때만)
/control_command (String) ──────► │ (팀원 상태기계: STOP, ESTOP … 그대로 전달)
                                  └─► /stm32_status, /bridge_status (COMM_OK / COMM_FAILED)
```

* 시리얼 포트는 이 노드만 연다. 팀원 `delivery_robot_bridge stm32_serial_bridge` 와 **같이 띄우지 않는다**
  (그 노드의 STM32 쪽 기능을 포함한다. ESP32 쪽 `esp32_serial_bridge` 는 그대로 쓴다).
* 줄 형식·펌웨어 요구 사항: `docs/stm32_drive_protocol.md`

## 실행

```bash
sudo apt install python3-serial          # rosdep 으로도 설치됨
ros2 launch robot_base base.launch.py port:=/dev/ttyACM0      # 실물 STM32
ros2 launch robot_base base.launch.py mock:=true              # 가짜 STM32 (펌웨어 없이)
```

robot.launch.py 는 따로 띄운다. IMU 가 없으면 EKF 를 바퀴 전용으로:
`ekf_params_file:=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/ekf_wheel_only.yaml`

## 바퀴 계산 (robot_base/swerve.py, `steer_mode: front`)

* 뒷바퀴는 0° 고정이라 회전 중심은 뒷차축 줄 위에 있다. 명령 (vx, wz) 는 뒷차축 중심 속도로 해석한다
  (차체 중심의 옆 속도 vy = wz × wheel_x 는 오도메트리에 그대로 나온다).
* 바퀴 i 속도 벡터 = (vx − wz·y_i, vy + wz·x_i) → 앞바퀴 각은 왼/오 따로(애커먼), 바퀴 속도 4개도 따로.
* 조향 ±30° 이면 최소 회전반경 1.07 m(뒷차축 중심), 1.09 m(차체 중심). 제자리 회전(뒷차축 중심)은
  59.5° 가 필요해서 불가 → 그런 명령은 정지, 너무 급한 곡선은 회전을 줄인다(로그 `명령 조정`).
* 조향이 목표에서 `steer_gate_deg`(10°) 이상 떨어져 있으면 바퀴를 굴리지 않고 조향부터 맞춘다.
  그 안이면 cos(오차) 만큼만 굴린다.
* 바퀴 하나라도 `max_wheel_rpm` 을 넘으면 네 바퀴를 같은 비율로 줄인다(회전 중심 유지).
* 오도메트리: 측정한 4바퀴 (조향각, 속도) 의 최소제곱 → vx, vy, wz. 잔차가 크면 바퀴끼리
  맞지 않는다는 뜻(heartbeat 의 `residual`).
* `steer_mode: all` 은 4바퀴 조향용(지금 실물 아님).

## 시뮬레이션

`robot.launch.py sim:=true` 는 `steer_sim_node` 를 같이 띄워 Gazebo 스키드 모델에 같은 조향 제한을 씌운다
(`sim_steer_limit:=false` 로 끔). 설정은 `config/base.yaml` 의 `/**` 공통값.

## 하드웨어 없이 시험

```bash
# 터미널 1
ros2 launch robot_base base.launch.py mock:=true
# 터미널 2: robot_safety 없이 직접 (실제 흐름에서는 /cmd_vel_safe 는 robot_safety 만 낸다)
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /wheel/odom --field twist.twist      # linear.x ≈ 0.2
ros2 topic echo /estop_state                         # data: false
```

mock 옵션: `--max-steer-deg 60`, `--steer-mode all`, `--no-steer-feedback`, `--estop-file /tmp/estop`
(파일 내용 `1` = E-stop 눌림), `--noise 1`.
