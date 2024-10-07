import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import threading

class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher_ = self.create_publisher(String, 'keyboard_button', 10)
        self.current_key = None
        self.lock = threading.Lock()

        self.publish_thread = threading.Thread(target=self.publish_key)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def publish_key(self):
        rate = self.create_rate(100)  # 10ms = 100Hz
        while rclpy.ok():
            with self.lock:
                if self.current_key:
                    msg = String()
                    msg.data = self.current_key
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published key: {self.current_key}')  # Thêm log tại đây
            rate.sleep()

    def on_press(self, key):
        try:
            with self.lock:
                self.current_key = key.char
                self.get_logger().info(f'Key pressed: {self.current_key}')  # Thêm log tại đây
        except AttributeError:
            with self.lock:
                self.current_key = str(key)
                self.get_logger().info(f'Special key pressed: {self.current_key}')  # Thêm log tại đây

    def on_release(self, key):
        with self.lock:
            self.current_key = None
        if key == keyboard.Key.esc:
            return False

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardListener()

    listener = keyboard.Listener(on_press=node.on_press, on_release=node.on_release)
    listener.start()

    rclpy.spin(node)

    listener.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
