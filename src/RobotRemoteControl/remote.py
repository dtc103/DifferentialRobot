import rclpy
from rclpy.node import Node
from pynput import keyboard

from std_msgs.msg import Int32

class Remote(Node):
    def __init__(self):
        super().__init__("remote_publisher")
        self.direction_timer = self.create_timer(1/50, self.direction_callback)
        self.yaw_timer = self.create_timer(1/20, self.yaw_callback)
        self.direction_publisher = self.create_publisher(Int32, "robot_direction", 10)
        self.yaw_publisher = self.create_publisher(Int32, "robot_yaw", 10)

        self.keys_pressed = set()

        self.listener = keyboard.Listener(
            on_press = self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def direction_callback(self):
        msg = Int32()

        if len(self.keys_pressed) == 0:
            msg.data = 0
            self.direction_publisher.publish(msg)
            print("PUBLISH STOP")
        elif keyboard.Key.up in self.keys_pressed:
            msg.data = 1
            self.direction_publisher.publish(msg)
            print("PUBLISH UP")
        elif keyboard.Key.down in self.keys_pressed:
            msg.data = -1
            self.direction_publisher.publish(msg)
            print("PUBLISH DOWN")

    def yaw_callback(self):
        msg = Int32()

        if keyboard.Key.left in self.keys_pressed:
            msg.data = -1
            self.yaw_publisher.publish(msg)
            print("PUBLISH LEFT")
        elif keyboard.Key.right in self.keys_pressed:
            msg.data = 1
            self.yaw_publisher.publish(msg)
            print("PUBLISH RIGHT")

    def on_press(self, key):
        self.keys_pressed.add(key)

        if key == keyboard.Key.esc:
            self.get_logger().info('ESC pressed, shutting down...')
            self.listener.stop()
            self.destroy_publisher(self.direction_publisher)
            rclpy.shutdown()

    def on_release(self, key):
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def __del__(self):
        if hasattr(self, 'listener') and self.listener.running:
            self.listener.stop()

def main():
    rclpy.init()
    node = Remote()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()