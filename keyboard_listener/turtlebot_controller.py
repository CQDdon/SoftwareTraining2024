import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Subscriber để nhận dữ liệu từ topic 'keyboard_button'
        self.subscription = self.create_subscription(
            String,
            'keyboard_button',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher để gửi dữ liệu vận tốc đến topic '/cmd_vel'
        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Tạo message Twist để lưu trữ vận tốc
        self.cmd_vel_msg = Twist()

        # Thời gian cho lần nhấn phím cuối cùng
        self.last_keypress_time = self.get_clock().now()

        # Thời gian tối đa giữa các lần nhận phím, sau đó sẽ dừng robot
        self.timeout = 0.011  # 0.5 giây

        # Tạo Timer để kiểm tra việc dừng robot khi không có lệnh mới
        self.create_timer(0.01, self.check_timeout)

    def listener_callback(self, msg):
        key = msg.data

        # Reset vận tốc
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0

        # Xử lý các ký tự điều khiển
        if key == 'w':
            self.cmd_vel_msg.linear.x = 0.5  # Tiến thẳng
        elif key == 'a':
            self.cmd_vel_msg.angular.z = 0.5  # Rẽ trái
        elif key == 's':
            self.cmd_vel_msg.linear.x = -0.5  # Lùi
        elif key == 'd':
            self.cmd_vel_msg.angular.z = -0.5  # Rẽ phải

        # Cập nhật thời gian nhấn phím cuối
        self.last_keypress_time = self.get_clock().now()

        # Publish lệnh vận tốc
        self.velocity_publisher.publish(self.cmd_vel_msg)

    def check_timeout(self):
        # Tính toán thời gian hiện tại và thời gian nhấn phím cuối
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_keypress_time).nanoseconds / 1e9  # Chuyển từ nano giây sang giây

        # Nếu quá timeout, dừng robot
        if time_diff > self.timeout:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)

    # Tạo node và chạy
    turtlebot_controller = TurtleBotController()
    rclpy.spin(turtlebot_controller)

    # Shutdown khi kết thúc
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
