
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')

        # ✅ 조건: 예를 들어, 항상 True로 설정 (원하면 다른 조건으로 변경 가능)
        self.should_beep = True

        # ✅ 최대 반복 횟수 및 현재 횟수
        self.max_beep_count = 3
        self.current_beep_count = 0

        self.publisher = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 10)
        self.timer = self.create_timer(1.0, self.publish_beep)

    def publish_beep(self):
        if not self.should_beep:
            self.get_logger().info('🔇 조건 미충족: 삐뽀 미재생')
            return

        if self.current_beep_count >= self.max_beep_count:
            self.get_logger().info('✅ 3회 재생 완료. 타이머 중지')
            self.timer.cancel()  # 타이머 중지로 더 이상 실행되지 않음
            return

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
        ]
        self.publisher.publish(msg)
        self.get_logger().info(f'🔊 삐-뽀-삐-뽀 재생 ({self.current_beep_count + 1}/{self.max_beep_count})')
        self.current_beep_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from irobot_create_msgs.msg import AudioNoteVector, AudioNote
# from builtin_interfaces.msg import Duration

# class BeepNode(Node):
#     def __init__(self):
#         super().__init__('beep_node')
        
#         self.publisher = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 10)
#         self.timer = self.create_timer(1.0, self.publish_beep)
        
#     def publish_beep(self):    
#         msg = AudioNoteVector()
#         msg.append = False
#         msg.notes = [
#         AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
#         AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
#         AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
#         AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
#         ]
#         self.publisher.publish(msg)
#         self.get_logger().info('🔊 삐-뽀-삐-뽀 published to /robot2/cmd_audio')

# def main(args=None):
#     rclpy.init(args=args)
#     node = BeepNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
