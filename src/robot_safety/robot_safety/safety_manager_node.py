# ============================================================
# safety_manager_node.py
# ============================================================
#
# 역할:
#
# 자율주행 알고리즘에서 만들어진 속도 명령을
# 그대로 모터로 보내지 않고,
#
# 1. 비상정지 상태
# 2. LiDAR 장애물 상태
#
# 를 확인한 뒤 안전한 명령만 출력한다.
#
#
# 입력:
#
# /cmd_vel_auto
#   자율주행 알고리즘이 만든 속도 명령
#
# /estop_state
#   비상정지 상태
#
# /safety/obstacle_detected
#   LiDAR 장애물 검출 결과
#
#
# 출력:
#
# /cmd_vel_safe
#
#
# 구조:
#
# waypoint_follower / Nav2
#          │
#          ▼
#   /cmd_vel_auto
#          │
#          ▼
# ┌───────────────────┐
# │ Safety Manager    │
# │                   │
# │ E-stop ?          │
# │ Obstacle ?        │
# └─────────┬─────────┘
#           │
#           ▼
#    /cmd_vel_safe
# ============================================================


# ROS2 Python API
import rclpy

# ROS2 Node 기본 클래스
from rclpy.node import Node

# 로봇 속도 명령 메시지
from geometry_msgs.msg import Twist

# True / False 메시지
from std_msgs.msg import Bool


# ============================================================
# SafetyManagerNode
# ============================================================
class SafetyManagerNode(Node):

    def __init__(self):

        # ROS2 node 이름
        super().__init__('safety_manager_node')


        # ====================================================
        # 현재 안전 상태 저장
        # ====================================================

        # E-stop
        #
        # False:
        # E-stop 눌리지 않음
        #
        # True:
        # E-stop 작동
        self.estop_active = False


        # 장애물 상태
        #
        # 처음에는 LiDAR 상태를 모르므로
        # fail-safe로 True
        self.obstacle_detected = True


        # ====================================================
        # 최근 자율주행 속도 명령
        # ====================================================
        #
        # Twist()를 생성하면 기본값은 전부 0
        #
        # 즉:
        #
        # linear.x = 0
        # angular.z = 0
        #
        # 정지 명령이다.
        self.latest_auto_cmd = Twist()


        # ====================================================
        # Subscriber 1
        # /cmd_vel_auto
        # ====================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_auto',
            self.cmd_callback,
            10
        )


        # ====================================================
        # Subscriber 2
        # /estop_state
        # ====================================================

        self.estop_sub = self.create_subscription(
            Bool,
            '/estop_state',
            self.estop_callback,
            10
        )


        # ====================================================
        # Subscriber 3
        # LiDAR 장애물 상태
        # ====================================================

        self.obstacle_sub = self.create_subscription(
            Bool,
            '/safety/obstacle_detected',
            self.obstacle_callback,
            10
        )


        # ====================================================
        # Publisher
        # ====================================================

        self.safe_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10
        )


        # 시작 로그
        self.get_logger().info(
            '[Safety] safety manager started'
        )


    # ========================================================
    # /cmd_vel_auto callback
    # ========================================================
    def cmd_callback(self, msg):

        # 최신 자율주행 명령 저장
        self.latest_auto_cmd = msg

        # 현재 안전 조건을 적용해서 publish
        self.publish_safe_command()


    # ========================================================
    # /estop_state callback
    # ========================================================
    def estop_callback(self, msg):

        # msg.data에는 True 또는 False가 들어있음
        self.estop_active = msg.data


        # E-stop이 눌리면 로그
        if self.estop_active:

            self.get_logger().warn(
                '[Safety] E-STOP ACTIVE'
            )


        # 상태가 변경됐으므로
        # 바로 속도 명령 다시 계산
        self.publish_safe_command()


    # ========================================================
    # /safety/obstacle_detected callback
    # ========================================================
    def obstacle_callback(self, msg):

        # LiDAR node에서 보내준 장애물 상태 저장
        self.obstacle_detected = msg.data


        # 장애물 상태가 변경되었으므로
        # 바로 안전 명령 계산
        self.publish_safe_command()


    # ========================================================
    # 최종 안전 속도 결정
    # ========================================================
    def publish_safe_command(self):

        # ----------------------------------------------------
        # E-stop 또는 장애물이 있으면
        # ----------------------------------------------------
        #
        # Python의 or:
        #
        # A or B
        #
        # 둘 중 하나라도 True면 전체가 True
        # ----------------------------------------------------

        if self.estop_active or self.obstacle_detected:

            # 새로운 Twist 메시지 생성
            #
            # 기본값이 모두 0이므로 정지 명령
            safe_cmd = Twist()


        # ----------------------------------------------------
        # 모든 안전조건이 정상이라면
        # ----------------------------------------------------
        else:

            # 자율주행 알고리즘의 명령을 그대로 통과
            safe_cmd = self.latest_auto_cmd


        # 최종 안전 속도 publish
        self.safe_cmd_pub.publish(
            safe_cmd
        )


# ============================================================
# main()
# ============================================================
def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)


    # SafetyManagerNode 객체 생성
    node = SafetyManagerNode()


    try:

        # ROS2 node 계속 실행
        rclpy.spin(node)

    except KeyboardInterrupt:

        pass


    # node 종료
    node.destroy_node()

    # ROS2 종료
    rclpy.shutdown()


# Python 파일을 직접 실행했을 때
if __name__ == '__main__':

    main()