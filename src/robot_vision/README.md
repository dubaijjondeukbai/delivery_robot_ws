# robot_vision

Raspberry Pi 5 배달로봇의 **perception 패키지**(ROS 2, `ament_python`).
카메라 입력 → AprilTag / QR / YOLO 결과를 ROS 2 토픽으로 publish 하는 것까지가 역할이며,
`/cmd_vel`, STOP, STM32/ESP32 제어는 **절대 하지 않는다**(robot_safety / robot_bridge 담당).

```
                        Camera (USB UVC | mock)
                                │
                                ▼
                          camera_node
                 ┌──────────────┴───────────────┐
                 ▼                              ▼
        /camera/image_raw               /camera/camera_info
        ┌────────┼─────────┐                    │
        ▼        ▼         ▼                    │
  apriltag_node qr_node detector_node           │
        │        │         │                    │
        ▼        ▼         ▼                    │
 /vision/tag_id /vision/ /vision/detections     │
        │       qr_result                       │
        └─── /vision/tag_pose ◀─────────────────┘
```

| Topic | Type | Publisher | 비고 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | camera_node (Pi Camera Module 3 Wide / USB / mock) | sensor QoS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_node | image와 동일 stamp. 미캘리브레이션 시 K/D/R/P = 0 |
| `/vision/tag_id` | `std_msgs/String` | apriltag_node | 선택된 tag id (예: `"3"`) |
| `/vision/tag_pose` | `geometry_msgs/PoseStamped` | apriltag_node | **캘리브레이션 완료 시에만** publish |
| `/vision/qr_result` | `std_msgs/String` | qr_node | repeat suppression 적용 |
| `/vision/detections` | `vision_msgs/Detection2DArray` | detector_node | 추론마다 publish (빈 배열 포함) |
| `/vision/debug_image` | `sensor_msgs/Image` | detector_node | `publish_annotated_image: true` 일 때만 |
| `/vision/apriltag_debug_image` | `sensor_msgs/Image` | apriltag_node | `publish_debug_image: true` 일 때만 |

## 패키지 구조

```
robot_vision/
├── config/vision.yaml          # 모든 노드의 파라미터 (TODO: tune on the real robot 주석 참고)
├── resource/robot_vision
├── robot_vision/
│   ├── __init__.py             # 비어 있음 (find_packages 용, 삭제 금지)
│   ├── common.py               # 토픽 이름 단일 정의 + ROS 독립 헬퍼 (quaternion, 이미지 변환)
│   ├── camera_node.py
│   ├── apriltag_node.py
│   ├── qr_node.py
│   └── detector_node.py
├── package.xml / setup.py / setup.cfg / README.md   (.gitignore 는 workspace 루트에 하나만 둔다)
```

## 1. 의존성 설치

아래는 Ubuntu 26.04 + ROS 2 Lyrical 기준이다. 다른 distro면 `${ROS_DISTRO}` 가 알아서 바뀐다.

```bash
source /opt/ros/lyrical/setup.bash          # 설치된 distro 에 맞게
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-rqt-image-view \
  python3-opencv python3-numpy v4l-utils qrencode

# 또는 package.xml 기준으로 자동 설치 (ultralytics 제외)
cd ~/delivery_robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 1-1. Ultralytics (YOLO) — pip 의존성

`ultralytics` 는 rosdep 키가 없으므로 `package.xml` 에 넣지 않고 pip 으로 설치한다.
ROS 2 노드는 **시스템 `/usr/bin/python3`** 로 실행되므로 그 인터프리터에 설치해야 한다
(venv/pyenv/conda 로 `python3` 가 바뀌어 있으면 rclpy import 가 깨진다).

```bash
which python3                                   # /usr/bin/python3 이어야 함
python3 -c "import numpy; print(numpy.__version__)"   # 설치 전 numpy 버전 기록
python3 -m pip install --break-system-packages -U ultralytics
python3 -c "import numpy; print(numpy.__version__)"   # major 버전이 바뀌었으면 아래 참고
python3 -c "from ultralytics import YOLO; import torch; print('torch', torch.__version__)"
```

주의 사항

* **numpy ABI**: `cv_bridge` 와 apt `python3-opencv` 는 시스템 numpy 의 major 버전에 맞춰 컴파일되어 있다.
  pip 이 numpy 의 major 버전을 바꿔 버리면(1.x ↔ 2.x) `import cv_bridge` 가
  `numpy.core.multiarray failed to import` 류의 에러로 죽는다. 그 경우 설치 전 버전과 같은 major 로 고정한다.
  예) 시스템이 1.26 이었으면 `python3 -m pip install --break-system-packages "numpy<2"`,
  시스템이 2.x 였으면 `"numpy>=2,<3"`.
* **Python 3.14 / aarch64 wheel**: Ubuntu 26.04 의 Python 3.14 용 torch wheel(특히 Pi 5 = aarch64)이
  없으면 `pip install ultralytics` 가 torch 단계에서 실패한다. 그때는 `vision.yaml` 의
  `detector_node.enabled: false` 로 두면 나머지 파이프라인은 그대로 동작한다.
  (`detector_node` 는 ultralytics 가 없어도 ERROR 로그만 내고 살아 있도록 되어 있다.)
* `opencv-python` 이 pip 으로 같이 깔려 apt 버전을 가리는 것은 문제없다(코드가 두 ArUco API 를 모두 지원).

### 1-2. 모델 준비

```bash
mkdir -p ~/models && cd ~/models
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"      # 현재 디렉터리에 다운로드
# (선택) Pi 5 CPU 용 NCNN export — 보통 PyTorch CPU 보다 빠르다
yolo export model=yolo11n.pt format=ncnn imgsz=416                  # -> ~/models/yolo11n_ncnn_model/
```

그 뒤 `vision.yaml` 의 `model_path` 를 **절대 경로**로 바꾼다
(`/home/<user>/models/yolo11n.pt` 또는 `/home/<user>/models/yolo11n_ncnn_model`).
상대 이름(`yolo11n.pt`)을 그대로 두면 `ros2 run` 을 실행한 디렉터리에 다운로드/탐색한다.
더 새로운 모델(예: `yolo26n.pt`)도 같은 방식으로 `model_path` 만 바꾸면 된다.

## 2. 빌드

```bash
cd ~/delivery_robot_ws
colcon build --packages-select robot_vision --symlink-install
source install/setup.bash
```

`--symlink-install` 을 쓰면 `.py`/`.yaml` 수정 후 재빌드 없이 바로 반영된다(파일 추가/삭제, setup.py 변경 시에는 재빌드).

## 3. 노드 단독 실행

```bash
source ~/delivery_robot_ws/install/setup.bash
export VISION_YAML=$(ros2 pkg prefix robot_vision)/share/robot_vision/config/vision.yaml

ros2 run robot_vision camera_node   --ros-args --params-file $VISION_YAML
ros2 run robot_vision apriltag_node --ros-args --params-file $VISION_YAML
ros2 run robot_vision qr_node       --ros-args --params-file $VISION_YAML
ros2 run robot_vision detector_node --ros-args --params-file $VISION_YAML

# 파라미터 즉석 override 예
ros2 run robot_vision camera_node --ros-args --params-file $VISION_YAML -p use_camera:=true -p camera_id:=0
```

## 4. 토픽 확인 명령

```bash
ros2 topic list
ros2 topic hz   /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /camera/image_raw --no-arr --once        # 픽셀 배열 생략
ros2 topic echo /vision/tag_id
ros2 topic echo /vision/tag_pose
ros2 topic echo /vision/qr_result
ros2 topic echo /vision/detections
ros2 run rqt_image_view rqt_image_view                   # 이미지 눈으로 확인
```

`/camera/*` 와 debug image 는 sensor QoS(best effort) 다. 최신 `ros2 topic echo` 는 publisher QoS 에 자동으로 맞추지만,
아무것도 안 보이면 `--qos-reliability best_effort` 를 붙인다.

## 5. 테스트 절차

### TEST 1 — Mock camera (`use_camera=false`)

```bash
ros2 run robot_vision camera_node --ros-args --params-file $VISION_YAML -p use_camera:=false
# 다른 터미널
ros2 topic hz /camera/image_raw            # ≈ 15 Hz
ros2 topic echo /camera/camera_info --once # width 640, height 480, k 가 모두 0 (미캘리브레이션)
```

heartbeat 로그: `[Vision] camera alive | mode=mock | backend=mock | camera_open=True | frames_published=... | capture_failures=0 | fps=15.0 | calibrated=False`.

### TEST 2 — 실제 카메라 (`use_camera=true`)

전방 카메라는 **Raspberry Pi Camera Module 3 Wide**(IMX708, CSI, 102° HFOV, 오토포커스)입니다.
CSI 카메라는 `/dev/videoN`이 아니라 **libcamera**로 서비스되므로 `camera_backend` 로 입력 경로를 고릅니다
(`auto` = picamera2 → gstreamer → v4l2 순서로 프레임이 나오는 첫 번째).

Pi 5(Ubuntu) 준비:

```bash
# libcamera 동작 확인 (카메라가 잡히고 프리뷰/프레임이 나와야 함)
sudo apt install -y libcamera-tools rpicam-apps          # 둘 중 설치되는 것
cam -l                                                   # 또는 rpicam-hello --list-cameras
# 경로 A) picamera2 (권장)
sudo apt install -y python3-picamera2 || python3 -m pip install --break-system-packages picamera2
python3 -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"
# 경로 B) GStreamer libcamerasrc (picamera2 가 안 깔릴 때)
sudo apt install -y gstreamer1.0-libcamera gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-tools
gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480,framerate=15/1 ! videoconvert ! fakesink -v
python3 -c "import cv2; print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])"   # YES 여야 함
```

실행:

```bash
ros2 run robot_vision camera_node --ros-args --params-file $VISION_YAML -p use_camera:=true
# 백엔드를 고정하려면: -p camera_backend:=picamera2  (또는 gstreamer / v4l2)
ros2 run rqt_image_view rqt_image_view               # /camera/image_raw 선택
```

* 로그에 `Camera opened: picamera2 camera_num=0 (imx708_wide 640x480 @ 15 fps RGB888)` 처럼 선택된 백엔드가 나와야 합니다.
* 색이 뒤바뀌어 보이면(빨강↔파랑) `picamera2_format: BGR888` 로 바꿉니다.
* 640×480(4:3)은 16:9 센서를 가로로 잘라 HFOV가 약 86°가 됩니다. 102° 전체를 쓰려면 `image_width: 640, image_height: 360`.
* 케이블을 뽑으면 `capture_failures` 가 늘고 2 초마다 재오픈을 시도하며, 다시 꽂으면 복구됩니다(노드는 죽지 않는다).
* USB 웹캠으로 임시 테스트할 때는 `-p camera_backend:=v4l2 -p camera_id:=0` (`ls /dev/video*`, `v4l2-ctl --list-devices`).
* **WSL2 에서는 카메라가 보이지 않습니다.** 실제 카메라 테스트는 Pi 5 에서 합니다.

### TEST 3 — AprilTag

1. tag36h11 마커를 출력한다(예: AprilRobotics `apriltag-imgs` 저장소의 `tag36h11/tag36_11_00000.png` 등).
   **검은 정사각형의 바깥쪽 한 변 길이**를 자로 재서 `apriltag_node.tag_size` (m) 에 넣는다.
2. ```bash
   ros2 run robot_vision camera_node   --ros-args --params-file $VISION_YAML -p use_camera:=true
   ros2 run robot_vision apriltag_node --ros-args --params-file $VISION_YAML
   ros2 topic echo /vision/tag_id        # data: "0"
   ```
3. 캘리브레이션 전: `tag_id` 만 나오고 로그에 10 초마다 `camera is not calibrated: tag_pose is not published` 경고.
4. 캘리브레이션 후(`camera_node.intrinsics.*` 입력): `ros2 topic echo /vision/tag_pose` 에서
   `position.z` 가 실제 거리(m)와 맞는지, 정면 태그의 `orientation` 이 대략 `(x≈1, y≈0, z≈0, w≈0)` 인지 확인한다.
   (태그의 z축이 카메라 쪽을 향하므로 180° 회전으로 표현된다.)
5. 눈으로 확인하려면 `-p publish_debug_image:=true` 후 `/vision/apriltag_debug_image` 를 rqt_image_view 로 본다.

### TEST 4 — QR

```bash
qrencode -s 10 -o station_a.png STATION_A      # 또는 아무 QR 생성기
ros2 run robot_vision qr_node --ros-args --params-file $VISION_YAML
ros2 topic echo /vision/qr_result             # data: STATION_A
```

QR 을 계속 비춰도 1 초(`repeat_suppression_sec`)에 한 번만 publish 되고, heartbeat 의 `suppressed` 가 늘어난다.

### TEST 5 / 6 — YOLO person 검출, 빈 결과

```bash
ros2 run robot_vision detector_node --ros-args --params-file $VISION_YAML
ros2 topic echo /vision/detections
```

* 사람이 카메라 앞에 서면 `detections[].results[0].hypothesis.class_id: "0"` (person), `bbox.center.position.x/y`, `size_x/y` 가 채워진다.
* 아무것도 없으면(또는 mock camera 의 검은 프레임이면) `detections: []` 인 빈 `Detection2DArray` 가 **계속** publish 된다
  → downstream 은 "객체 없음" 과 "detector 사망" 을 메시지 유무로 구분할 수 있다.
* 박스를 눈으로 보려면 `-p publish_annotated_image:=true` 후 `/vision/debug_image`.

### TEST 7 — YOLO 가 카메라보다 느릴 때 backlog 가 없는지

```bash
# 15 Hz 카메라 + 1 Hz 추론으로 일부러 느리게
ros2 run robot_vision detector_node --ros-args --params-file $VISION_YAML -p inference_rate_hz:=1.0
```

heartbeat 에서 `frames_received` 는 ≈15/s, `inferences` 는 ≈1/s, `dropped_frames` 가 ≈14/s 로 **선형**으로 증가하고
메모리/지연이 쌓이지 않으면 정상이다. `last_inference_ms` 를 보고 `inference_rate_hz` 를 실제 처리 속도보다 조금 낮게 잡는다.

## 6. 설계 메모

* **Backlog 방지(detector_node)**: image callback 은 최신 프레임 1 장만 보관하고, 별도 inference timer 가 그 프레임을 처리한다.
  구독과 추론은 서로 다른 MutuallyExclusiveCallbackGroup + 2-thread executor 에서 돌아 추론 중에도 프레임이 계속 갱신(=drop)된다.
* **Calibration 정책**: `camera_node` 가 `K[0]==0` 으로 미캘리브레이션을 알리고(`sensor_msgs/CameraInfo` 규약),
  `apriltag_node` 는 그 경우 `tag_id` 만 publish 하고 경고는 10 초마다 1 회로 제한한다.
* **OpenCV API 호환**: ArUco 신/구 API(`ArucoDetector` vs `detectMarkers`), `SOLVEPNP_IPPE_SQUARE` 유무,
  `detectAndDecodeMulti` 유무, `vision_msgs` 4.0/4.1 의 `BoundingBox2D.center` 레이아웃 차이를 코드에서 흡수한다.
* **확장성**: 토픽 이름은 `common.py` + 파라미터로 관리하고 카메라 입력은 `camera_node` 의 FrameSource 클래스
  (picamera2 / gstreamer / v4l2)로 분리되어 있어 카메라를 바꿔도 인식 노드는 손대지 않는다. YOLO 는 `DetectionBackend` 인터페이스 뒤에 있어 NCNN/가속기 교체 시 그 클래스만 바뀐다.
  `Detection2D.id` 는 비워 두어 ByteTrack 도입 시 tracking id 로 쓴다.

## 7. 주의

* `vision.yaml` 에서 double 파라미터는 반드시 소수점을 붙인다(`15.0`). 정수 파라미터에는 붙이지 않는다.
* `package.xml` / `setup.py` 의 maintainer, license 는 기존 skeleton 값으로 되돌려 둘 것(TODO 표시).
* 모든 노드는 매 프레임 로그를 찍지 않는다. 이벤트(태그 선택/상실, QR 디코드)와 heartbeat(기본 5 s, `[Vision] <node> alive | ...`)만 출력한다.
* `apriltag_node` / `qr_node` / `detector_node` 는 `enabled: false` 로 두면 이미지 구독 없이 heartbeat 만 찍는다(robot.yaml 에서 시스템 레벨로 끌 수 있음).
