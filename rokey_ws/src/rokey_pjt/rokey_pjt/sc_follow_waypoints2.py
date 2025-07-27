#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time
from rclpy.qos import qos_profile_sensor_data


def create_pose(x, y, yaw_deg, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class ParkingLocationCommander(Node):
    def __init__(self):
        super().__init__('parking_location_commander')

        self.navigator = BasicNavigator()
        self.dock_navigator = TurtleBot4Navigator()

        self.location_map = {
            "A-1": (-2.28, -5.01, -90.0),
            "A-2": (-1.32, -5.15, -90.0),
            "B-1": (1.03, -2.06, 0.0),
            "B-2": (0.94, -1.10, 0.0),
            "C-1": (-2.95, -3.54, 180.0),
            "C-2": (-3.04, -4.59, 180.0),
        }

        self.initial_xyyaw = (-0.02, -0.02, 0.0)
        self.wait_xyyaw = (-1.03, -0.02, 0.0)

        self.go_to_initial_pose()

        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('🚦 현재 도킹 상태 → 언도킹 수행')
            self.dock_navigator.undock()
            time.sleep(2.0)

        self.subscription = self.create_subscription(
            String,
            '/parking/location',
            self.location_callback,
            10
        )
        self.get_logger().info('✅ Subscribed to /parking/location')

        self.parking_coord = None

        self.create_subscription(
            PoseStamped,
            '/detect/object_map_pose',
            self.object_map_pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('✅ Subscribed to /detect/object_map_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

    def object_map_pose_callback(self, msg):
        self.parking_coord = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.orientation
        )
        self.get_logger().info(
            f"📍 Received object map pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

        if self.parking_coord:
            self.get_logger().warn("====================주차 시작")

            obj_x = self.parking_coord[0]
            obj_y = self.parking_coord[1]
            obj_yaw = 0.0
            obj_pose = create_pose(obj_x, obj_y, obj_yaw, self.navigator)
            # 동기로 삐삐삐 
            self.go_to_pose_blocking(obj_pose, "객체 기반 주차 위치")

            time.sleep(2)
            self.get_logger().warn("====================주차 완료")
        else:
            self.get_logger().warn("⚠️ 객체 맵 좌표 없음")

        # ✅ 정확한 180도 회전 로직
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # rad/s

        duration = 3.141592 / 0.5  # π rad / 속도 ≒ 6.28초
        self.get_logger().info(f"↪️ 제자리 180도 회전 시작 (예상 {duration:.2f}초)")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("✅ 회전 완료, 로봇 정지!")
        # beep 띠리링
        time.sleep(3.0)

        self.go_to_wait_pose()
        self.get_logger().info('대기위치 복귀 완료')

        # 2️⃣ 초기 위치 복귀
        initial_pose = create_pose(*self.initial_xyyaw, self.navigator)
        self.go_to_pose_blocking(initial_pose, "초기 위치 복귀")

        # 3️⃣ 도킹 요청
        self.get_logger().info('🚀 초기 위치 도착 → 도킹 요청 시작')
        self.dock_navigator.dock()
        self.get_logger().info('✅ 도킹 요청 완료')

    def go_to_pose_blocking(self, pose, description):
        self.get_logger().info(f"🚗 이동 시작: {description}")
        self.navigator.goToPose(pose)
        start_time = self.navigator.get_clock().now()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                elapsed = self.navigator.get_clock().now() - start_time
                self.get_logger().info(
                    f"➡️ 진행 중 - 남은 거리: {feedback.distance_remaining:.2f} m, "
                    f"경과 시간: {elapsed.nanoseconds / 1e9:.1f} s"
                )
            time.sleep(0.2)

    def go_to_initial_pose(self):
        initial = create_pose(*self.initial_xyyaw, self.navigator)
        self.navigator.setInitialPose(initial)
        self.get_logger().info('✅ AMCL 초기 Pose 설정 완료')
        time.sleep(1.0)
        self.navigator.waitUntilNav2Active()

    def go_to_wait_pose(self):
        wait_pose = create_pose(*self.wait_xyyaw, self.navigator)
        self.go_to_pose_blocking(wait_pose, "대기 지점")
        time.sleep(5.0)

    def location_callback(self, msg):
        self.go_to_wait_pose()

        location = msg.data.strip()
        self.get_logger().info(f"📌 Received parking command: {location}")

        if location not in self.location_map:
            self.get_logger().warn(f"❗ Unknown location: {location}")
            return

        x, y, yaw = self.location_map[location]
        target_pose = create_pose(x, y, yaw, self.navigator)
        self.go_to_pose_blocking(target_pose, f"지정 주차 위치: {location}")
        time.sleep(3.0)


def main():
    rclpy.init()
    commander = ParkingLocationCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.get_logger().info('🛑 종료 요청 감지')
    finally:
        commander.navigator.lifecycleShutdown()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
