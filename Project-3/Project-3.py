import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.time import Duration

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State machine variables
        self.state = "turn"
        self.state_start_time = self.get_clock().now()
        
        # Tunable parameters (calibrate for your robot)
        self.angular_speed = 0.5  # rad/s (45° in 1.57s)
        self.linear_speed = 0.2   # m/s (2m in 10s)
        self.stop_buffer = 0.5    # Buffer time to ensure full stop

    def control_loop(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.state_start_time
        twist = Twist()

        if self.state == "turn":
            # Turn counterclockwise
            twist.angular.z = self.angular_speed
            if elapsed_time > Duration(seconds=1.57):  # π/4 radians at 0.5 rad/s
                self.state = "stop_before_move"
                self.state_start_time = current_time
                # Publish zero twist to halt
                self.cmd_vel_pub.publish(Twist())
        
        elif self.state == "stop_before_move":
            # Wait for robot to fully stop
            if elapsed_time > Duration(seconds=self.stop_buffer):
                self.state = "move"
                self.state_start_time = current_time
        
        elif self.state == "move":
            # Move forward
            twist.linear.x = self.linear_speed
            if elapsed_time > Duration(seconds=10.0):  # 2m at 0.2 m/s
                self.state = "stop"
                self.cmd_vel_pub.publish(Twist())
        
        elif self.state == "stop":
            self.destroy_node()
            rclpy.shutdown()
        
        # Publish velocity command for current state
        if self.state not in ["stop_before_move", "stop"]:
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)

if __name__ == '__main__':
    main()