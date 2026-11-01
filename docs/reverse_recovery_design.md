# 후진 복구 설계 (현재 비활성 유지)

작성 2026-11-01. **결론: 후방 감지가 검증되지 않았으므로 자동 후진은 계속 비활성.** 이 문서는 도입 조건과
설계만 정리한다. 코드/설정은 바꾸지 않았다.

## 1. 현재 상태 확인

| 위치 | 값 | 의미 |
|---|---|---|
| `robot_safety/config/safety.yaml` | `max_reverse_velocity: 0.0` | Safety 가 음수 `linear.x` 를 0 으로 자름(각속도는 통과) |
| `safety_manager_node.py` | `max(-max_reverse_velocity, ...)` | 위와 동일, 최종 권한 |
| `robot_navigation/behavior_trees/nav_to_pose_replan_on_invalid_path.xml` | `BackUp` 주석 처리 | 복구 순환에서 후진 제거 |
| `nav2.yaml` behavior_server | `backup` 플러그인은 로드됨 | BT 가 부르지 않으면 동작 안 함 |
| `nav2.yaml` velocity_smoother | `min_velocity: [-0.1, 0.0, -0.5]` | 음수 통과 가능하지만 Safety 에서 0 |
| RPP | `allow_reversing: false` | 경로 추종 중 후진 없음 |
| `lidar_obstacle_node` | 전방 ±35° 만 검사 | **후방 검사 없음** |
| 뚜껑(CAD) | 열리면 LiDAR 후방 약 157° 가림 | 열린 상태 후방 데이터 무효 |

클라우드 시뮬레이션(Jazzy 기본 BT, BackUp 포함)에서 BackUp/DriveOnHeading 은 Safety 가 후진을 0 으로 잘라
`Exceeded time allowance` 로 끝났다. 사용자 BT 에서 BackUp 을 뺀 판단은 현재 Safety 정책과 맞다.

## 2. "제자리 회전이 어렵다"만으로 후진을 켜지 않는 이유
* 앞바퀴 조향 실물에서 회전이 안 되는 문제는 **경로 계획(최소 회전반경)** 으로 풀어야 한다
  (`drive_kinematics_gap.md`). 후진은 그 다음 수단이다.
* 후방에는 검증된 감지가 없다. 후진 허용은 Safety 의 최종 정책 변경이며, 정지 판단과 같은 수준의 검증이 필요하다.

## 3. 도입 시 설계: 실제 주행한 궤적 되짚기

### 3.1 궤적 기록 (odometry breadcrumb)
* 입력: `/odom`(EKF, odom 프레임). 과거 **계획 경로가 아니라 실제로 지나온 자세**를 쓴다.
* 0.05 m 이상 이동하거나 0.1 rad 이상 회전할 때마다 자세를 추가, 최근 3 m / 60 s 만 유지.
* odom 리셋·점프(EKF reset, 시간 역행)가 감지되면 기록을 비운다(되짚을 근거가 없어짐).

### 3.2 시작 조건 (모두 만족해야 함)
1. 후방 감지 검증 완료 + 뚜껑 닫힘 인터록(`/cargo/lid_open`=false, 최근 수신).
2. 후방 영역 스캔이 유효(데이터 비율·신선도, 뚜껑/차체 가림 각도 제외 후).
3. E-stop 정상, 위치 추정 정상(TF 조회 성공, AMCL 공분산 임계 이하).
4. 전방이 막혀(`obstacle_in_stop_zone`) 진행 불가이고, 계획/회전 복구로 해결되지 않은 경우에만
   (BT 에서 FollowPath 105/106 실패 후, 재계획 실패 다음 순서).
5. 기록된 궤적이 되짚을 거리 이상 남아 있음.

### 3.3 동작 제한
* 최대 거리: 0.5 m(설정, 상한 1.0 m). 최대 속도: 0.10 m/s. 최대 시간: `거리/속도 × 2 + 2 s`.
* 궤적에서 벗어남 0.15 m 초과 시 중단. 회전은 궤적 방향을 따를 때만(앞바퀴 조향이면 후진 조향).

### 3.4 중단 조건 (하나라도 발생하면 즉시 정지)
* 후방 정지 영역 안 장애물, 후방 스캔 무효/지연, 뚜껑 열림, E-stop, TF/odom 점프, 시간·거리 초과,
  목표 취소·변경, Safety STOP(어떤 사유든).

### 3.5 끝난 뒤 순서
1. 0.5 s 정지 → 2. 로컬 코스트맵 비우기 → 3. 현재 위치에서 재계획(ComputePathToPose) →
4. FollowPath 재개. 같은 목표에서 되짚기 최대 2회, 이후 목표 실패로 보고.

### 3.6 권한 구조
* 후진 허용은 BT 가 아니라 **Safety 가 결정**: 후방 검사 노드가 `/safety/reverse_permitted`(Bool, 주기 발행)를
  내고, Safety 는 그 값이 신선하고 true 일 때만 `max_reverse_velocity`(예: 0.10) 를 적용, 아니면 0.
* 구현 경로: `nav2_msgs/action/BackUp` 형식의 Python 액션 서버(`backtrack`)를 만들어 궤적을 되짚게 하고,
  BT 의 `<BackUp server_name="backtrack" .../>` 로 호출하면 C++ BT 플러그인 없이 연결 가능.

## 4. 검증 순서 (도입 전 필수)
1. 후방 LiDAR 데이터 확인(뚜껑 닫힘/열림 각각, 차체·아치 가림 각도 측정).
2. 시뮬레이션: 후방 장애물 배치 시 중단 조건 전부 시험, `motion_chain_monitor` 로 명령 흐름 기록.
3. 실물: 바퀴 들고(벤치) → 저속 실주행 → 사람 없는 공간 순서.
4. 이 단계들을 통과하기 전에는 `max_reverse_velocity: 0.0` 과 BT 의 BackUp 주석을 유지한다.
