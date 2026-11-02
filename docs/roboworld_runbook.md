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
1. 처음 위치: **출발점을 바닥에 테이프로 표시**하고, 처음 한 번만 RViz `2D Pose Estimate` 로 맞춘 뒤
   `ros2 run robot_bringup stations record HOME` 으로 저장한다. 그다음부터는 로봇을 테이프 위에 두고
   T3 에 `initial_station:=HOME` 을 붙이면 켜질 때 위치가 맞춰진다(RViz 필요 없음).
   실행 중에 다시 맞출 때(로봇을 들어서 옮겼을 때): `ros2 run robot_bringup stations init HOME`.
2. 지점 저장: 로봇을 시연 동선대로 손으로/teleop 으로 몰면서 지점마다
   `ros2 run robot_bringup stations record p1` … `p4`. **방향 = 지나가는 방향**으로 저장.
   (teleop 은 Nav2 와 같은 `/cmd_vel_auto` 를 쓰므로, 지점 저장할 때는 `nav_mode:=localization`
   으로 띄우거나 Nav2 목표가 없을 때만 조종한다.)
3. 순회: `ros2 run robot_bringup stations tour p1 p2 p3 p4 --loop --dwell 5`  (Ctrl+C = 목표 취소·정지)
4. 한 곳만: `ros2 run robot_bringup stations go p2`

### 동선 규칙 (앞바퀴 조향 ±60°, 경로는 전진만)
* 최소 회전반경(차체 중심) 약 0.6 m, Nav2 경로는 0.7 m 로 계획(`nav2.yaml minimum_turning_radius`).
  ±60° 는 실측 전 값이다(서보 링크 때문에 바퀴 각이 다를 수 있음) → 실측해서 `base.yaml max_steer_deg` 수정.
* **U턴은 폭 3 m 이상 빈 곳에서만**, 되도록 한 방향으로 도는 고리 동선(앞 0.8 m 정지 구역 때문).
* 벽·부스 모서리에서 **1 m 이상** 떨어진 동선. 앞 0.8 m(±35°) 안에 물체가 있으면 Safety 가 멈춘다.
* 사람이 앞 0.8 m 안에 들어오면 멈췄다가, 비키면 다시 간다(정상).
* **후진 탈출** (`reverse_escape:=true`, `demo:=true` 에 포함): 막혀서 Nav2 가 실패하면 첫 복구로
  0.8 m 를 0.08 m/s 로 곧게 후진한 뒤 다시 계획한다. Safety 는 뒤쪽 LiDAR 영역(뒤 ±35°, LiDAR 에서 1.4 m
  = 차체 뒤끝에서 약 0.74 m)이 비어 있을 때만 통과시키고, 후진 중에 누가 들어오면 바로 멈춘다.
  뚜껑이 열려 있으면 뒤가 가려져 후진하지 않는다. 처음에 실물에서 뒤에 아무것도 없을 때
  `ros2 topic echo /safety/rear_distance` 가 1.4 보다 큰지 확인(작으면 로봇 자기 몸이 보이는 것 → 알려 주세요).
* 시뮬레이션(클라우드, Jazzy, 벽 0.67 m 앞에서 뒤쪽 목표):
  - 후진 탈출 없이: 로봇이 전혀 움직이지 못하고 30 s 만에 목표 실패.
  - 후진 탈출 켬: 0.8 m 후진 → 벽에서 빠져나와 목표 근처(0.4 m 안)까지 갔다. 다만 목표 방향 맞추기에서
    맴돌아 시간 안에 완료하지 못했다(시뮬은 Lyrical 전용 BT 노드가 없어 매초 재계획하는 기본 트리를 썼다).
  - 뒤 1.1 m 에 벽이 있을 때 후진 명령: Safety 가 `STOP:rear_obstacle_detected` 로 막음, 이동 0 m.
  → 벽 바로 앞에서 되돌아 나오는 동선은 여전히 피한다(위 규칙).

## 4-1. QR 배달 시연 (출발 QR → 이동 → 정지 → 도착 QR → 뚜껑 열기)

QR 내용(문자열 그대로 QR 코드로 만든다): `START:A` (A 로 출발), `OPEN:A` (A 에서 뚜껑 열기), `CLOSE` (닫기).
지점 이름 = QR 의 A/B (영문·숫자·`_`·`-`, 40자 이내).

```bash
# 준비: 4 의 T3 대신. demo:=true = vision:=qr camera:=true lid_interlock:=true reverse_escape:=true
# (하나만 끄려면 예: reverse_escape:=false 를 덧붙인다). 출발점 HOME 은 4 의 1번 참고.
ros2 launch robot_bringup robot.launch.py nav_mode:=navigation motion_enabled:=true demo:=true \
  initial_station:=HOME map:=$HOME/maps/roboworld.yaml ekf_params_file:=$EKF_WHEEL       # T3
# 지점 저장 (처음 한 번): 로봇을 A 위치·도착 방향으로 놓고
ros2 run robot_bringup stations record A        # B 도 같은 방법. ~/maps/stations.yaml 에 저장
# 시연 노드 + ESP32 브리지(팀원 esp32_serial_bridge) + Pi 모니터 화면
ros2 launch robot_bringup delivery_demo.launch.py esp32_port:=/dev/serial/by-id/<ESP32> display:=true  # T5
ros2 run robot_bringup delivery_console                                                        # T6 운영자 콘솔
```

Pi 모니터 화면(`display:=true`, `delivery_display`): 미션 상태를 한글 큰 글씨로 띄운다
(대기 / A 지점으로 이동 중 / 도착 QR 안내 / 뚜껑 열림 / 오류 이유와 복구 방법, 위 띠에 장애물·E-stop,
아래 띠에 뚜껑 상태·마지막으로 읽은 QR·시각). 처음 한 번 Pi 에서:
`sudo apt install python3-pygame fonts-nanum`. 데스크톱 없는 Ubuntu Server 면 콘솔 화면에 바로 그린다
(사용자가 `video` 그룹이어야 함: `sudo usermod -aG video $USER` 후 다시 로그인). 데스크톱이 있으면 전체 화면
창으로 뜨고, SSH 로 띄울 때는 `DISPLAY=:0` 을 앞에 붙인다. 화면에서 Esc/q 로 끈다(시연 노드는 계속 돈다).
노트북에서 미리 보기: `display:=true display_fullscreen:=false`.

흐름과 규칙 (`/delivery/state` 의 `state`):
1. `IDLE` 에서 카메라에 `START:A` → `NAVIGATING`. 이동 중에는 `OPEN`·다른 `START` 를 무시한다.
2. Nav2 도착 → STM32 에 STOP → `/odom` 이 0.5 s 동안 0 **이고** STM32 `DRIVE_STOPPED` → `WAIT_OPEN`.
   5 s 안에 확인이 안 되면 `ERROR`(뚜껑 안 엶). STM32 펌웨어가 `DRIVE_STOPPED` 를 아직 못 보내면
   T5 에 `require_drive_stopped:=false` 를 붙인다(`/odom` 정지만 확인).
3. `OPEN:A` (도착지와 같을 때만) → ESP32 `LID_OPEN`. 리미트 스위치가 없으므로 `OPEN_UNCONFIRMED`
   (열림, 센서 확인 안 됨)로 표시된다 → **눈으로 확인**. 도착지가 다른 `OPEN:B` 는 열지 않는다.
4. 물건을 꺼낸 뒤 `CLOSE` QR (또는 운영자 콘솔 `c`) → `IDLE` (뚜껑 `CLOSED_UNCONFIRMED`). 다음 `START` 가능.
   같은 `START:A` 를 계속 보여 줘도 끝난 뒤 3 s 동안은 다시 출발하지 않는다.

운영자 콘솔 (`ros2 run robot_bringup delivery_console`, QR 없이 명령 + 상태 보기):
`a`/`b` = START:A/B, `oa`/`ob` = OPEN:A/B, `c` = CLOSE, `x` = CANCEL(이동 취소), `r` = RESET(오류 해제),
`s` = 상태 다시 보기, `START:C` 처럼 그대로 입력해도 된다, `q` 끝. QR 과 같은 규칙이 적용된다.
* `ERROR` 가 나면: 원인 해결 → 뚜껑이 열려 있거나 상태가 `UNKNOWN` 이면 `CLOSE` → `RESET`.
  RESET 은 뚜껑을 닫혔다고 가정하지 않는다(뚜껑이 닫힘이 아니면 START 를 거부하고 CLOSE 를 안내).
* ESP32 를 켜기 전에 뚜껑을 손으로 닫아 둔다(스위치가 없어 켜질 때 닫혀 있다고 본다).
* **뚜껑 인터록** (`lid_interlock:=true`): 시연 노드가 뚜껑을 닫힘으로 알고 있을 때만 Safety 가 바퀴를
  움직이게 한다. 열림·여닫는 중·모름이면 teleop·Nav2 명령 모두 `STOP:lid_open`.
  `delivery_demo` 가 안 떠 있으면 `STOP:waiting_for_lid_state`(먼저 T5 를 띄운다).
  뚜껑 모터가 고장 나 CLOSE 가 안 되면: 손으로 닫고 T5(`delivery_demo`)를 다시 시작(켜질 때 닫힘으로 봄).
  지도 만들기·벤치 시험처럼 시연 노드 없이 움직일 때는 `lid_interlock` 을 빼고 실행한다.

펌웨어 없이 먼저 시험 (가짜 뚜껑, 시뮬):
```bash
ros2 launch robot_bringup robot.launch.py sim:=true nav_mode:=navigation motion_enabled:=true demo:=true
ros2 launch robot_bringup delivery_demo.launch.py sim:=true mock_lid:=true display:=true display_fullscreen:=false
ros2 run robot_bringup delivery_console          # a → (도착 후) oa → c
```

## 4-2. 노트북에서 조종하기 (Pi 4 가 전부 실행, 노트북은 명령·확인만)

**가장 확실한 방법: SSH** (네트워크 설정이 필요 없다). 노트북과 Pi 를 같은 와이파이에 두고
(전시장 와이파이 대신 노트북 핫스팟이나 휴대용 공유기 권장) PowerShell 또는 WSL 에서:
```bash
ssh <pi사용자>@<pi IP>           # Pi IP: Pi 에서 hostname -I
tmux new -s demo                # 와이파이가 끊겨도 실행 중인 launch 가 죽지 않게 tmux 안에서 실행
#   새 창 Ctrl+b c, 창 이동 Ctrl+b 숫자, 빠져나오기 Ctrl+b d, 다시 붙기: tmux attach -t demo
```
tmux 창마다 위 T1~T6 을 실행하고, 콘솔(`delivery_console`)과 teleop 도 SSH 안에서 쓴다.
(`sudo apt install tmux`. tmux 없이 SSH 로 띄운 launch 는 SSH 가 끊기면 같이 죽는다.)

**RViz 를 노트북 WSL 에서 보고 싶으면** (선택, 미리 집에서 확인):
1. Windows 11 22H2 이상에서 `%USERPROFILE%\.wslconfig` 에 `[wsl2]` 아래 `networkingMode=mirrored` → PowerShell `wsl --shutdown`
2. 관리자 PowerShell: `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`
3. 노트북 WSL 과 Pi 에 같은 `export ROS_DOMAIN_ID=<숫자>`, 둘 다 `ROS_LOCALHOST_ONLY` 는 설정하지 않는다.
4. 노트북에서 `ros2 topic echo /delivery/state` 가 보이면 성공 → `rviz2`, `delivery_console` 도 노트북에서 된다.
   안 보이면 노트북에 `export ROS_STATIC_PEERS=<pi IP>`, Pi 에 `export ROS_STATIC_PEERS=<노트북 IP>`(같은 터미널들에 모두).
5. RViz 에 TF·"dropping message" 경고가 계속 나면 두 기기의 시계가 어긋난 것 → Pi 를 인터넷에 연결해 시간 동기화
   (`timedatectl` 로 확인).
이 방법은 와이파이가 끊기면 노트북 쪽 명령·화면만 끊기고, 주행·안전은 Pi 안에서 계속 돈다.

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
| `STOP:waiting_for_lid_state` / `STOP:lid_state_timeout` | `lid_interlock:=true` 인데 `delivery_demo` 가 안 떠 있음 |
| `STOP:lid_open` | 뚜껑이 닫힘이 아님 → `CLOSE` (모터 고장이면 손으로 닫고 `delivery_demo` 재시작) |
| `STOP:rear_obstacle_detected` | 후진 탈출 중 뒤에 물체(1.4 m 안) → 비키면 다음 복구에서 다시 시도 |
| 뒤에 아무것도 없는데 후진을 안 함 | `/safety/rear_distance` 확인: 1.4 이하면 자기 몸·뚜껑이 보이는 것 |
| `initial_station 'HOME' is not in ...` | `stations record HOME` 을 먼저 (2D Pose Estimate 로 맞춘 뒤) |
| 카메라 화면이 검정, QR 안 읽힘 | T3 에 `camera:=true` 를 안 붙임 |
| 모니터에 "연결 대기" | `delivery_demo` 가 안 떠 있음 / 다른 `ROS_DOMAIN_ID` |
| 모니터 위 띠 "Safety 신호 없음" | `robot.launch.py` 가 안 떠 있음 |
| 모니터 글자가 네모 | 한글 폰트 없음: `sudo apt install fonts-nanum` |
| `[Display] 화면을 열 수 없음` | Server: `video` 그룹·다시 로그인 / 데스크톱: `DISPLAY=:0` 붙여서 실행 |
| 콘솔에 "[상태] 아직 못 받음" | 노트북에서 실행했다면 4-2 의 네트워크 설정, 아니면 Pi 에 SSH 로 들어가서 실행 |

기록: `ros2 run robot_bringup motion_chain_monitor --ros-args -p output_file:=$HOME/run_$(date +%H%M).jsonl`
