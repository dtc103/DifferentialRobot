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
        self.control_mode_publisher = self.create_publisher(Int32, "robot_control_mode", 10)

        self.keys_pressed = set()
        self.current_mode = 0  # 0=PID, 1=Stanley, 2=MPC
        self.mode_names = {0: "PID", 1: "Stanley", 2: "MPC"}

        self.listener = keyboard.Listener(
            on_press = self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        print("Remote Control Ready")
        print("  Arrow keys: direction and yaw")
        print("  1: PID mode | 2: Stanley mode | 3: MPC mode")
        print("  ESC: quit")
        print(f"  Current mode: {self.mode_names[self.current_mode]}")

    def direction_callback(self):
        msg = Int32()

        if len(self.keys_pressed) == 0:
            msg.data = 0
            self.direction_publisher.publish(msg)
        elif keyboard.Key.up in self.keys_pressed:
            msg.data = 1
            self.direction_publisher.publish(msg)
        elif keyboard.Key.down in self.keys_pressed:
            msg.data = -1
            self.direction_publisher.publish(msg)

    def yaw_callback(self):
        msg = Int32()

        if keyboard.Key.left in self.keys_pressed:
            msg.data = -1
            self.yaw_publisher.publish(msg)
        elif keyboard.Key.right in self.keys_pressed:
            msg.data = 1
            self.yaw_publisher.publish(msg)

    def publish_control_mode(self, mode):
        if mode < 0 or mode > 2:
            return
        self.current_mode = mode
        msg = Int32()
        msg.data = mode
        self.control_mode_publisher.publish(msg)
        print(f"Switched to: {self.mode_names[mode]}")

    def on_press(self, key):
        self.keys_pressed.add(key)

        # Controller mode switching: keys 1, 2, 3
        try:
            if hasattr(key, 'char'):
                if key.char == '1':
                    self.publish_control_mode(0)  # PID
                elif key.char == '2':
                    self.publish_control_mode(1)  # Stanley
                elif key.char == '3':
                    self.publish_control_mode(2)  # MPC
        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            self.get_logger().info('ESC pressed, shutting down...')
            self.listener.stop()
            self.destroy_publisher(self.direction_publisher)
            self.destroy_publisher(self.yaw_publisher)
            self.destroy_publisher(self.control_mode_publisher)
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
