# maps/

저장된 지도를 이 디렉터리에 두면 `colcon build` 시 `share/robot_navigation/maps/` 로 설치되고,
`localization.launch.py` / `navigation.launch.py` 의 `map` 인자 기본값
(`<share>/maps/exhibition_map.yaml`)으로 바로 사용된다.

## 지도 저장 (MODE A, mapping.launch.py 실행 중)

```bash
mkdir -p ~/maps
# 1) AMCL / map_server 용: map.yaml + map.pgm
ros2 run nav2_map_server map_saver_cli -f ~/maps/exhibition_map \
    --ros-args -p map_subscribe_transient_local:=true
# 2) (권장) slam_toolbox pose graph: 나중에 지도를 이어서 만들거나 수정할 때 필요
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$HOME/maps/exhibition_map'}"
```

결과 파일

| 파일 | 용도 |
|---|---|
| `exhibition_map.yaml` | map_server 가 읽는 메타데이터 (아래 참고) |
| `exhibition_map.pgm` | occupancy grid 이미지 (흰색 free, 검정 occupied, 회색 unknown) |
| `exhibition_map.posegraph`, `exhibition_map.data` | slam_toolbox 전용. AMCL 은 사용하지 않는다 |

## 패키지에 넣기

```bash
cp ~/maps/exhibition_map.yaml ~/maps/exhibition_map.pgm ~/delivery_robot_ws/src/robot_navigation/maps/
cd ~/delivery_robot_ws && colcon build --packages-select robot_navigation --symlink-install
```

## map.yaml 필드

```yaml
image: exhibition_map.pgm   # yaml 과 같은 디렉터리의 상대 경로. 두 파일을 항상 같이 옮길 것
mode: trinary
resolution: 0.05            # m/cell (slam.yaml 의 resolution 과 동일)
origin: [-10.2, -7.5, 0.0]  # 이미지 좌하단 픽셀의 map 좌표 (x, y, yaw)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## 주의

* 지도를 만든 뒤 전시장 집기 배치가 크게 바뀌면 AMCL 정합도가 떨어진다. 그 경우 지도를 다시 만들거나,
  `.posegraph` 를 `mode: localization` 의 slam_toolbox 로 불러 보정하는 방법이 있다(README 참고).
* 지도 원점(`origin`)이 바뀌면 그 지도 기준으로 저장해 둔 station 좌표도 모두 무효가 된다.
