# ============================================================
# lidar_obstacle_node.py
# ============================================================
#
# 역할:
#   LiDAR의 /scan 데이터를 받아서
#   로봇 전방에 장애물이 있는지 판단한다.
#
# 입력:
#   /scan
#   타입: sensor_msgs/msg/LaserScan
#
# 출력:
#   /safety/obstacle_detected
#   타입: std_msgs/msg/Bool
#
#   /safety/front_distance
#   타입: std_msgs/msg/Float32
#
# 판단 기준:
#   전방 ±35도
#   0.8 m 이내에 3개 이상의 LiDAR point가 있으면 STOP
#   1.0 m 밖으로 충분히 벗어나야 다시 CLEAR
#
#   /scan이 0.5초 이상 끊겨도 안전을 위해 STOP
# ============================================================


# Python의 수학 관련 기능을 사용하기 위한 모듈
# radians(), sin(), cos(), atan2(), isfinite() 등을 사용한다.
import math

# ROS2 Python 라이브러리
import rclpy

# ROS2 Node 클래스를 가져온다.
from rclpy.node import Node

# LiDAR 메시지 타입
from sensor_msgs.msg import LaserScan

# True / False 메시지 타입
from std_msgs.msg import Bool

# 실수(float) 메시지 타입
from std_msgs.msg import Float32


# ============================================================
# LidarObstacleNode 클래스
# ============================================================
#
# Python 문법:
#
# class 자식클래스(부모클래스):
#
# LidarObstacleNode가 ROS2의 Node를 상속받는다.
#
# 따라서 이 클래스 자체가 ROS2 Node가 된다.
# ============================================================
class LidarObstacleNode(Node):

    # --------------------------------------------------------
    # 생성자
    # --------------------------------------------------------
    #
    # __init__()은 객체가 생성될 때 자동으로 실행된다.
    # --------------------------------------------------------
    def __init__(self):

        # 부모인 Node의 생성자 실행
        #
        # ROS2 node 이름:
        # /lidar_obstacle_node
        super().__init__('lidar_obstacle_node')


        # ====================================================
        # Parameter 선언
        # ====================================================
        #
        # ROS2 Parameter는 나중에 yaml 파일이나 launch에서
        # 값을 변경할 수 있는 설정값이다.
        # ====================================================

        # 로봇 정면을 기준으로 좌우 35도씩 검사
        self.declare_parameter('front_angle_deg', 35.0)

        # 이 거리 이하에 장애물이 있으면 정지
        self.declare_parameter('stop_distance', 0.8)

        # 장애물 정지 상태에서 다시 움직이기 위한 거리
        self.declare_parameter('release_distance', 1.0)

        # 최소 3개의 point가 잡혀야 장애물로 인정
        self.declare_parameter('min_points', 3)

        # 0.5초 동안 LiDAR 데이터가 안 오면 센서 이상으로 판단
        self.declare_parameter('scan_timeout', 0.5)


        # ====================================================
        # Parameter 값 읽기
        # ====================================================

        self.front_angle_deg = (
            self.get_parameter('front_angle_deg')
            .get_parameter_value()
            .double_value
        )

        self.stop_distance = (
            self.get_parameter('stop_distance')
            .get_parameter_value()
            .double_value
        )

        self.release_distance = (
            self.get_parameter('release_distance')
            .get_parameter_value()
            .double_value
        )

        self.min_points = (
            self.get_parameter('min_points')
            .get_parameter_value()
            .integer_value
        )

        self.scan_timeout = (
            self.get_parameter('scan_timeout')
            .get_parameter_value()
            .double_value
        )


        # ====================================================
        # 내부 상태 변수
        # ====================================================

        # 처음에는 LiDAR가 정상인지 모르기 때문에
        # 안전을 위해 장애물이 있다고 가정한다.
        #
        # 이것을 fail-safe 방식이라고 한다.
        self.obstacle_detected = True

        # 마지막으로 LiDAR 데이터를 받은 시간
        #
        # Python의 None:
        # 아직 값이 없다는 의미
        self.last_scan_time = None

        # 현재 전방 최소 거리
        self.front_distance = float('inf')


        # ====================================================
        # Subscriber
        # ====================================================
        #
        # create_subscription(
        #     메시지 타입,
        #     topic 이름,
        #     callback 함수,
        #     queue 크기
        # )
        #
        # /scan 데이터가 들어올 때마다
        # self.scan_callback() 함수가 실행된다.
        # ====================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )


        # ====================================================
        # Publisher
        # ====================================================

        # 장애물 있음/없음을 publish
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/safety/obstacle_detected',
            10
        )

        # 전방에서 가장 가까운 장애물 거리 publish
        self.distance_pub = self.create_publisher(
            Float32,
            '/safety/front_distance',
            10
        )


        # ====================================================
        # Timer
        # ====================================================
        #
        # 0.1초마다 check_scan_timeout() 실행
        #
        # 즉 10 Hz
        # ====================================================

        self.timeout_timer = self.create_timer(
            0.1,
            self.check_scan_timeout
        )


        # 시작 메시지 출력
        self.get_logger().info(
            '[LiDAR] obstacle detector started | '
            f'ROI=±{self.front_angle_deg:.1f} deg | '
            f'stop={self.stop_distance:.2f} m | '
            f'release={self.release_distance:.2f} m'
        )


    # ========================================================
    # /scan callback
    # ========================================================
    #
    # LiDAR 데이터가 들어올 때마다 실행된다.
    #
    # msg에는 LaserScan 데이터가 들어온다.
    # ========================================================
    def scan_callback(self, msg):

        # 현재 시각 저장
        #
        # scan이 정상적으로 계속 들어오는지 확인하기 위해 사용
        self.last_scan_time = self.get_clock().now()


        # degree → radian 변환
        #
        # LiDAR 각도는 기본적으로 radian을 사용한다.
        front_angle_rad = math.radians(
            self.front_angle_deg
        )


        # 전방 영역의 거리 데이터만 저장할 list
        #
        # [] = Python list
        valid_front_ranges = []


        # ====================================================
        # LiDAR point 하나씩 검사
        # ====================================================
        #
        # enumerate():
        #
        # index와 값을 동시에 가져오는 Python 함수
        #
        # 예:
        #
        # ranges = [1.0, 2.0, 3.0]
        #
        # i = 0, distance = 1.0
        # i = 1, distance = 2.0
        # i = 2, distance = 3.0
        # ====================================================

        for i, distance in enumerate(msg.ranges):

            # NaN 또는 infinity 값 제거
            #
            # continue:
            # 현재 반복을 건너뛰고 다음 반복으로 이동
            if not math.isfinite(distance):
                continue

            # LiDAR 최소 측정거리보다 작으면 무시
            if distance < msg.range_min:
                continue

            # LiDAR 최대 측정거리보다 크면 무시
            if distance > msg.range_max:
                continue


            # 현재 point의 각도 계산
            #
            # angle_min:
            # 첫 번째 LiDAR point의 각도
            #
            # angle_increment:
            # point 하나당 증가하는 각도
            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )


            # 각도를 -pi ~ +pi 범위로 정규화
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )


            # 로봇 전방 ±35도 안에 있는 데이터만 사용
            #
            # abs():
            # 절댓값 함수
            if abs(angle) <= front_angle_rad:

                # list 끝에 distance 추가
                valid_front_ranges.append(distance)


        # ====================================================
        # 전방 데이터가 하나도 없는 경우
        # ====================================================

        if len(valid_front_ranges) == 0:

            # 센서 상태가 불확실하므로 STOP
            self.update_obstacle_state(
                True,
                float('inf'),
                'no valid front scan'
            )

            # 함수 종료
            return


        # ====================================================
        # 가장 가까운 point 거리
        # ====================================================

        min_distance = min(valid_front_ranges)

        self.front_distance = min_distance


        # ====================================================
        # 장애물이 없는 상태
        # ====================================================
        #
        # Python:
        #
        # if not False:
        #     → True
        #
        # 따라서 obstacle_detected가 False일 때 실행
        # ====================================================

        if not self.obstacle_detected:

            # 0.8m 이하의 point 개수 계산
            #
            # 아래는 Python generator 표현식
            close_points = sum(
                1
                for r in valid_front_ranges
                if r <= self.stop_distance
            )


            # 가까운 point가 3개 이상이면
            if close_points >= self.min_points:

                # 장애물 있음으로 변경
                self.update_obstacle_state(
                    True,
                    min_distance,
                    'obstacle entered stop zone'
                )

            else:

                # 상태 변화 없음
                self.publish_state()


        # ====================================================
        # 이미 장애물 때문에 STOP 상태
        # ====================================================
        else:

            # 이번에는 release_distance = 1.0m를 사용
            close_points = sum(
                1
                for r in valid_front_ranges
                if r <= self.release_distance
            )


            # 1.0m 안쪽 point가 3개 미만이면
            # 장애물이 충분히 멀어졌다고 판단
            if close_points < self.min_points:

                self.update_obstacle_state(
                    False,
                    min_distance,
                    'obstacle cleared'
                )

            else:

                self.publish_state()


    # ========================================================
    # LiDAR timeout 확인
    # ========================================================
    def check_scan_timeout(self):

        # LiDAR를 한 번도 받은 적이 없는 경우
        if self.last_scan_time is None:

            self.obstacle_detected = True
            self.publish_state()

            return


        # 현재 시각
        now = self.get_clock().now()


        # 시간 차이 계산
        #
        # nanoseconds / 1e9
        #
        # → 초(second) 단위로 변환
        elapsed = (
            now - self.last_scan_time
        ).nanoseconds / 1e9


        # 0.5초 이상 LiDAR가 끊긴 경우
        if elapsed > self.scan_timeout:

            # 이전에는 주행 가능 상태였다면
            if not self.obstacle_detected:

                self.get_logger().error(
                    '[LiDAR] scan timeout -> FAIL-SAFE STOP'
                )

            # 안전을 위해 장애물 있음 상태
            self.obstacle_detected = True

            self.publish_state()


    # ========================================================
    # 장애물 상태 변경 함수
    # ========================================================
    def update_obstacle_state(
        self,
        new_state,
        distance,
        reason=''
    ):

        # 기존 상태와 새로운 상태가 다른지 확인
        #
        # !=
        # "같지 않다"라는 Python 비교 연산자
        state_changed = (
            new_state != self.obstacle_detected
        )


        # 새로운 상태 저장
        self.obstacle_detected = new_state

        self.front_distance = distance


        # 상태가 실제로 바뀐 경우에만 로그 출력
        if state_changed:

            # True면 장애물 감지
            if new_state:

                self.get_logger().warn(
                    f'[LiDAR] STOP | '
                    f'distance={distance:.2f} m | '
                    f'{reason}'
                )

            # False면 장애물 해제
            else:

                self.get_logger().info(
                    f'[LiDAR] CLEAR | '
                    f'distance={distance:.2f} m | '
                    f'{reason}'
                )


        # ROS2 topic으로 상태 publish
        self.publish_state()


    # ========================================================
    # 상태 publish 함수
    # ========================================================
    def publish_state(self):

        # Bool 메시지 생성
        obstacle_msg = Bool()

        # 메시지의 data에 True/False 저장
        obstacle_msg.data = self.obstacle_detected

        # publish
        self.obstacle_pub.publish(obstacle_msg)


        # Float32 메시지 생성
        distance_msg = Float32()

        # Python float으로 변환해서 저장
        distance_msg.data = float(
            self.front_distance
        )

        # publish
        self.distance_pub.publish(distance_msg)


# ============================================================
# main()
# ============================================================
#
# setup.py에서
#
# lidar_obstacle_node =
# robot_safety.lidar_obstacle_node:main
#
# 이라고 등록했기 때문에 이 함수가 실행된다.
# ============================================================
def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)


    # LidarObstacleNode 객체 생성
    #
    # 이 순간 __init__() 실행
    node = LidarObstacleNode()


    try:

        # 노드를 계속 실행
        #
        # subscriber callback과 timer callback이
        # 계속 처리된다.
        rclpy.spin(node)

    # Ctrl+C 입력 시 발생
    except KeyboardInterrupt:

        # 아무 작업 없이 넘어감
        #
        # pass = Python에서 "아무것도 하지 않음"
        pass


    # node 종료
    node.destroy_node()

    # ROS2 종료
    rclpy.shutdown()


# ============================================================
# 직접 실행된 파일인지 확인
# ============================================================
#
# Python 파일을 직접 실행하면
#
# __name__ == '__main__'
#
# 이 된다.
# ============================================================
if __name__ == '__main__':

    main()