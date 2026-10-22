#!/usr/bin/env python3
"""
[Vision] pose_node

목적:
    detector_node.py의 영상 수신·추론·추적·발행 기능을
    사람 Pose 모드로 실행한다.

입력:
    /camera/image_raw (sensor_msgs/msg/Image)

기본 출력:
    /vision/pose_boxes (vision_msgs/msg/Detection2DArray)
    /vision/poses (std_msgs/msg/String, 관절 정보 JSON)
    /vision/pose_debug_image (sensor_msgs/msg/Image, 설정 시)

기본 모델:
    yolo11n-pose.pt

관절 좌표를 발행하며, 행동 위험 판단·모터 제어는 하지 않는다.
모델 경로·입출력 토픽·추론 주기는 ROS 파라미터로 변경할 수 있다.
"""

import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)
from rclpy.logging import get_logger

from robot_vision.detector_node import DetectorNode


def main(args=None) -> None:
    # ROS2 초기화
    rclpy.init(args=args)

    # 생성 도중 오류가 나더라도 종료 처리할 수 있도록 초기화
    node = None
    executor = None

    try:
        # [핵심] 이름과 기능 모드를 별도로 지정한다.
        # pose_mode=True:
        # 사람 Pose 모델과 Pose 출력 토픽을 기본으로 선택
        node = DetectorNode(
            node_name='pose_node',
            pose_mode=True,
        )

        # 이미지 수신과 모델 추론을 서로 다른 스레드에서 처리
        # callback group 분리는 DetectorNode 내부에 구현되어 있다.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 또는 외부 ROS 종료 요청
        pass

    except Exception as exc:
        get_logger('pose_node').fatal(
            f'[Vision] pose_node failed to start or crashed: {exc!r}'
        )
        raise

    finally:
        # 실행기 → 노드 → ROS 순서로 종료
        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


# Python 파일을 직접 실행했을 때
if __name__ == '__main__':
    main()