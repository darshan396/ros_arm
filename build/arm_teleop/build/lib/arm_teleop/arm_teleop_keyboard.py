import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('arm_teleop_keyboard')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("=== Robotic Arm Keyboard Control ===")
        self.get_logger().info("W/S: Shoulder joint up/ddown")
        self.get_logger().info("A/D: Elbow joint up/down") 
        self.get_logger().info("O: Open claw")
        self.get_logger().info("C: Close claw")
        self.get_logger().info("SPACE: Reset all joints")
        self.get_logger().info("Q: Quit")
        self.get_logger().info("================================")

        # Initialize joint positions
        self.shoulder_joint = 0.0
        self.elbow_joint = 0.0
        self.claw_joint = 0.0

        # Claw states
        self.CLAW_OPEN = 0.1
        self.CLAW_CLOSED = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def main_loop(self):
        msg = JointState()
        msg.name = ['shoulder_joint', 'elbow_joint', 'claw_joint']

        while True:
            key = self.get_key().lower()

            # Shoulder joint control
            if key == 'w':
                self.shoulder_joint += 0.1
                self.get_logger().info("Shoulder: UP")
            elif key == 's':
                self.shoulder_joint -= 0.1
                self.get_logger().info("Shoulder: DOWN")
            
            # Elbow joint control
            elif key == 'a':
                self.elbow_joint += 0.1
                self.get_logger().info("Elbow: UP")
            elif key == 'd':
                self.elbow_joint -= 0.1
                self.get_logger().info("Elbow: DOWN")
            
            # Claw control
            elif key == 'o':
                self.claw_joint = self.CLAW_OPEN
                self.get_logger().info("Claw: OPEN")
            elif key == 'c':
                self.claw_joint = self.CLAW_CLOSED
                self.get_logger().info("Claw: CLOSED")
            
            # Reset all joints
            elif key == ' ':
                self.shoulder_joint = 0.0
                self.elbow_joint = 0.0
                self.claw_joint = self.CLAW_OPEN
                self.get_logger().info("RESET: All joints to zero")
            
            # Quit
            elif key == 'q':
                self.get_logger().info("Shutting down...")
                break
            else:
                continue

            # Apply joint limits
            self.shoulder_joint = max(-3.14, min(3.14, self.shoulder_joint))
            self.elbow_joint = max(0.0, min(2.62, self.elbow_joint))
            self.claw_joint = max(0.0, min(0.1, self.claw_joint))

            # Publish joint states
            msg.position = [self.shoulder_joint, self.elbow_joint, self.claw_joint]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            
            self.get_logger().info(
                f"Current Positions - Shoulder: {self.shoulder_joint:.2f}, "
                f"Elbow: {self.elbow_joint:.2f}, "
                f"Claw: {self.claw_joint:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopKeyboard()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()