import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_robot)

    def move_robot(self):
        twist = Twist()
        twist.linear.x = 1.0  
        twist.angular.z = 0.5  
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
