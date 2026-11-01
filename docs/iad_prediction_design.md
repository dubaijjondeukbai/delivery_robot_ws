# IAD 보행자 경로 예측 — 검토 결과와 (기본 비활성) 연결 설계

작성 2026-11-01. 대상:
* 논문: *Intention-Aware Diffusion Model for Pedestrian Trajectory Prediction*, AAAI 2026
  (Yu Liu, Zhijie Liu, Xiao Ren, You-Fu Li, He Kong — SUSTech / CityU HK), arXiv 2508.07146
* 코드: https://github.com/AISLAB-sustech/IAD (2025-08-03 커밋 033b837 기준)

**결론: 지금은 연결하지 않는다.** 입력(지면 좌표 궤적)을 만들 수 없고, 공개 코드는 추론 경로가 그대로
실행되지 않으며 가중치도 없다. 아래 설계는 전제 조건이 갖춰진 뒤 "관찰 모드"로 시작하기 위한 것이다.

## 1. 확인한 사실

| 항목 | 논문 | 공개 코드 |
|---|---|---|
| 데이터 | ETH/UCY, SDD (위에서 내려다본 영상) | `raw_data/` 에 ETH/UCY txt, SDD pkl 포함 |
| 입력 좌표계 | 월드 좌표(미터), 결과도 미터 | 위치·속도·가속도(x, y) 표준화, Trajectron++ 장면 그래프(이웃 포함) |
| 관측/예측 길이 | 관측 8 + 예측 12 프레임(합 8 s → 0.4 s 간격: 3.2 s + 4.8 s) | `maximum_history_length: 7`, `prediction_horizon: 12` |
| 샘플링 | best-of-20, 확산 K=100, DDIM stride 20 | `sample=20`, `num_steps//5` |
| 구조 | 잡음 추정기 Transformer 4층(hidden 512, 4 head), 의도 추정 self-attention 4층(256) | 같은 구성 + RefineNet |
| 추론 시간 | **보고 없음** (실험 하드웨어 RTX 5090 / Xeon 8481C) | — |
| 가중치 | 언급 없음 | **없음** (`torch.load(...epoch.pt)` 경로만 있음, 설정 YAML·전처리 pkl 도 없음) |
| 라이선스 | — | MIT |
| 장치 | — | `.cuda()` 고정 → CPU 실행에 수정 필요 |

### 공개 코드의 추론 경로 문제
`models/autoencoder.py` 의 `generate()` 는 `self.goal_loss(y_t, goal, p_goal)` 을 부르는데
1) `goal_loss` 는 코드 어디에도 정의돼 있지 않고(정의된 것은 `goal_loss_v2`, 학습용),
2) 인자 `y_t` 는 **미래 정답 궤적**이다.
논문은 "추론 시 신뢰도가 가장 높은 끝점을 고른다"고 적었으므로, 쓰려면 이 부분을 `p_goal` 최댓값 선택으로
고쳐야 하고, 고친 뒤의 정확도는 논문 수치와 다를 수 있다(재평가 필요).

### 계산량(대리 측정)
같은 층 모양의 신경망(난수 가중치)을 만들어 예측 1회(20 샘플 × DDIM 5 단계 = 잡음 추정기 100회 +
RefineNet 100회, 인코더 제외) 시간을 쟀다. 클라우드 x86 Xeon 2.1 GHz, 2 스레드, PyTorch CPU:

| 보행자 수 | 공개 코드처럼 샘플 순차 | 20 샘플 묶음 처리 |
|---|---|---|
| 1 | 585 ms | 141 ms |
| 5 | 869 ms | 493 ms |
| 10 | 1418 ms | 982 ms |

Raspberry Pi 5 에서는 측정하지 않았다. 모델 기본 주기(0.4 s)를 Pi 에서 YOLO·Nav2 와 함께 맞추기는 어렵다고
보는 것이 안전하다. Pi 에서 같은 대리 측정: `python3 docs/iad_timing_proxy.py 4` (torch 필요).

## 2. 현재 비전 출력으로 IAD 입력을 만들 수 있는가

| IAD 요구 | 현재 상태 | 부족한 것 |
|---|---|---|
| 보행자별 **지면 위치(m)**, 정지 좌표계(map/odom) | bbox 픽셀 좌표, 미보정 단안 | 카메라 내부 보정, 장착 높이·pitch 실측, 지면 호모그래피 또는 LiDAR-카메라 융합 거리, TF 로 자기 운동 보정 |
| 0.4 s 간격 8 프레임(3.2 s) 연속 이력 | ByteTrack ID, 1 s 공백 시 추적기 리셋, 검출 목표 5 Hz(Pi 실측 미정) | 3.2 s 이상 끊기지 않는 ID, 재표본화 |
| 주변 보행자(이웃) 동시 이력 | 동시 검출은 가능 | 위와 같은 좌표·시간 조건 |
| 학습 분포와 비슷한 장면 | 0.1 m 높이 전방 카메라(지평선 근처) | 위에서 본 데이터로 학습된 모델 → 재학습/미세조정 데이터 필요 |
| 가중치 | 없음 | 직접 학습 필요(코드·설정 일부 누락) |

카메라가 바닥에서 약 0.10 m(추정)라 발 위치가 지평선 바로 아래에 모이고, 거리 1 m 와 4 m 의 화면 차이가
몇 % 에 불과하다(실측 pitch 오차에 매우 민감). **단안 화면 좌표만으로는 IAD 입력을 만들 수 없다.**
따라서 IAD 결과를 실제 위험 판정에 연결하지 않는다.

## 3. (나중) 연결 설계 — 기본 비활성, 관찰 모드부터

```
/vision/detections ─┐                                    ┌─> /prediction/pedestrians (관찰 기록)
LiDAR(/scan) ───────┼─> [person_tracker_3d] ─> 지면 궤적 ─┤
TF(map/odom) ───────┘    (거리 융합, 미구현)               └─> rosbag / CSV 평가
                          ↓ 0.4 s 재표본화, 8 프레임 이력
                     [trajectory_predictor_node]  mode: off(기본) | observe | advisory
```

* 노드: `trajectory_predictor_node` (별도 프로세스, `mode: off` 기본). observe 에서는 예측만 기록.
* 출력: 입력 stamp, 계산 시각, 보행자 ID, 20 샘플(또는 대표 3개), 유효 플래그, 모델 버전.
* 평가(관찰 모드): 지연시간(입력 stamp→출력), ADE/FDE(m, 실제 이후 궤적과 비교), 놓침
  (로봇 경로 영역에 들어온 사람 중 사전 예측이 없던 비율), 오탐(예측은 들어온다 했지만 안 들어온 비율).
  기준 예: 지연 < 0.4 s, 놓침 < 5 %(목표값은 팀에서 정함) — 통과 전에는 advisory 도 켜지 않는다.
* 고장 격리:
  * 예측 노드가 죽거나 늦어도 robot_safety 판단은 바뀌지 않는다(구독하지 않음).
  * 소비자는 "예측 없음/오래됨"을 "안전"으로 해석하지 않는다.
  * CPU 제한(`nice`, 스레드 수, 주기 제한), 예외 시 유효 플래그 false 로 발행.
  * 검증 후에도 Safety 에는 "추가 정지 사유"로만 넣고 LiDAR/E-stop 정지를 해제하거나 늦추는 데 쓰지 않는다.

## 4. 선행 작업 순서
1. 카메라 보정(내부), 장착 높이·pitch 실측 → `robot_params.xacro` 갱신.
2. LiDAR-카메라 정합(외부 보정) + 사람 거리 추정(군집/투영) → 지면 궤적 토픽.
3. 추적 연속성 개선(ID 유지 3 s 이상) 및 0.4 s 재표본화.
4. 캠퍼스 데이터 수집 → IAD 재학습 또는 더 가벼운 기준선(등속 모델, Social-LSTM 계열)과 비교.
5. 관찰 모드 평가 → 기준 통과 시 advisory.
