# 구동 방식 차이: 실물(앞바퀴 조향) vs Gazebo(스키드) vs Nav2 설정

작성 2026-11-01. 실물 구동 계획은 사용자 설명 기준이다:
**인휠 모터 4개(구동) + 앞바퀴 조향 서보 2개**. 2026-10-30 프로젝트 메모에는 "4륜 독립 조향(4WIS)"으로
적혀 있어 서로 다르다. 아래는 앞바퀴 조향(자동차형, Ackermann 계열) 기준이며, 4WIS 로 확정되면
"제자리 회전 가능" 여부부터 다시 판단해야 한다.

## 1. 지금 코드가 가정하는 운동 모델

| 위치 | 현재 설정 | 가정 |
|---|---|---|
| Gazebo `gazebo.xacro` | `DiffDrive` 플러그인, 좌/우 각 2개 바퀴, 조향 관절 없음 | 스키드(차동) — 제자리 회전 가능 |
| `/wheel/odom` | DiffDrive 가 바퀴 회전으로 계산 | 미끄럼 없는 차동 구동 |
| Nav2 planner | NavFn(격자 Dijkstra) | 운동 제약 없음(제자리 회전으로 어느 방향이든 출발 가능) |
| Nav2 controller | RPP `use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785` | 방향 오차가 크면 제자리 회전 |
| BT 복구 | `Spin 1.57`, `Wait`, (BackUp 제거) | 제자리 회전 가능 |
| AMCL | `DifferentialMotionModel` | 차동/자동차형 모두 사용 가능 |
| robot_safety | `linear.x`, `angular.z` 만 허용, v=0·ω≠0 도 통과 | 제자리 회전 명령을 막지 않음 |

## 2. 앞바퀴 조향 실물과의 차이

* **제자리 회전 불가**: v=0, ω≠0 명령은 물리적으로 실행할 수 없다. 최소 회전반경
  `R_min ≈ L / tan(δ_max)` (L = 축거, CAD 0.460 m, δ_max = 서보 최대 조향각 TODO).
  δ_max 가 30°면 약 0.80 m, 40°면 약 0.55 m (안쪽 바퀴 기준이 아닌 차체 중심 기준 근사).
* RPP 의 rotate-to-heading, BT 의 Spin 복구, 큰 방향 전환(예: 목표가 뒤쪽) 은 실물에서 그대로 실패한다
  → `Failed to make progress` / Spin 시간 초과로 나타난다.
* NavFn 경로는 곡률 제한을 모른다. 좁은 곳에서 R_min 보다 급한 꺾임을 만들 수 있다.
* `/wheel/odom` 의 계산식이 달라진다: 조향각 + 바퀴 속도(전자식 차동). 스키드 미끄럼 보정 대신
  조향각 오차·바퀴 슬립을 다뤄야 한다.

## 3. 시뮬레이션 문제와 실물 적용 과제 구분

### 시뮬레이션 문제 (이번에 확인/수정한 것)
1. **스키드 근사의 회전 미끄럼**: 명령 yaw rate 의 약 60~70%만 실제로 회전, `/wheel/odom` 의 yaw rate 는
   실제의 약 1.5배(클라우드 Jazzy+Harmonic 실측, Gazebo 실측 자세 비교). → EKF 는 IMU yaw rate 를
   쓰도록 원래 설정으로 되돌림(`robot_navigation/config/ekf.yaml`).
2. **바퀴 마찰 비등방성**: `mu1=1.0, mu2=0.5` 에 `fdir1` 이 없어 마찰 방향이 바퀴와 무관하게 정해졌고,
   제자리 회전량이 로봇 방향에 따라 1~71%로 들쭉날쭉했다. → `mu2=1.0`(등방성)으로 바꿈.
3. **시뮬레이션은 여전히 제자리 회전을 허용**한다. 따라서 시뮬레이션에서 성공한 목표가 실물(앞바퀴 조향)에서
   성공한다는 보장이 없다.

### 실물 적용 과제 (아직 안 한 것, 순서 제안)
1. 서보 최대 조향각 δ_max, 조향 응답 속도, 인휠 모터 정격 rpm 확정 → R_min, 최대 속도 계산.
2. 하위 제어(STM32 또는 ROS bridge)에서 `(v, ω)` → `(δ, 바퀴 4개 속도)` 변환. `|ω| > |v| / R_min` 인
   명령(제자리 회전 포함)은 실행 불가이므로 **어디서 거부/제한할지** 정한다(권장: bridge 에서 제한하고
   제한 사실을 상태 토픽으로 알림. robot_safety 는 정지 판단만 유지).
3. Nav2 를 자동차형으로 변경:
   * planner: Smac Hybrid-A*(`motion_model_for_search: DUBIN`, 후진 허용 시 `REEDS_SHEPP`) 또는 Smac Lattice,
     `minimum_turning_radius: R_min`.
   * controller: RPP `use_rotate_to_heading: false` (+ 후진은 후방 안전 확인 전까지 false),
     또는 MPPI `motion_model: Ackermann` + `AckermannConstraints.min_turning_r`.
   * BT: Spin 복구 제거(제자리 회전 불가), DriveOnHeading/후진 복구는 `reverse_recovery_design.md` 조건 충족 후.
   * goal checker yaw 허용 오차·progress checker 를 조향 차량 기준으로 다시 조정.
4. Gazebo 를 실물과 맞추기: `gz-sim-ackermann-steering-system` 과 앞바퀴 조향 관절(조향 링크) 추가,
   또는 그 전 단계로 "명령을 Ackermann 가능 범위로 바꾸는 시뮬레이션 전용 필터"를 두어 Nav2 자동차형 설정을
   먼저 시험.
5. EKF: 바퀴 vx(+vy=0 제약) + IMU vyaw 유지. 조향각 기반 yaw rate 를 넣을지는 실물 데이터로 판단.

## 4. 판단 요약
* 현재 시뮬레이션(스키드)과 Nav2 설정은 서로 일관되지만 **계획된 실물 조향 방식과는 맞지 않는다.**
* 이번 수정은 "시뮬레이션 안에서 추정과 실제가 어긋나는 문제"만 고쳤다. 앞바퀴 조향 대응은 하드웨어 수치
  (δ_max, 조향 속도, 축거 실측)가 정해진 뒤 3절 순서로 진행하는 것이 맞다.
