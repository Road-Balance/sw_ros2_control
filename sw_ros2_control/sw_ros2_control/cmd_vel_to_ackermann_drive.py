import math
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)

class AckermannConverter(Node):

    def __init__(self):
        super().__init__('ackermann_converter')

        self.twist_sub = self.create_subscription(Twist,
                        '/cmd_vel',
                        self.cmd_vel_callback,
                        10)
                                            
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, 
                        '/ackermann_commands',
                        10)

        self.ackermenn_msg = AckermannDriveStamped()
        self.wheelbase = 1.0
        self.frame_id = "odom"

    def cmd_vel_callback(self, msg):
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        steering = convert_trans_rot_vel_to_steering_angle(
            linear_x, angular_z, self.wheelbase)

        self.ackermenn_msg = AckermannDriveStamped()
        self.ackermenn_msg.header.stamp = self.get_clock().now().to_msg()
        self.ackermenn_msg.header.frame_id = self.frame_id
        self.ackermenn_msg.drive.steering_angle = float(steering)
        self.ackermenn_msg.drive.speed = float(linear_x)

        print(linear_x, steering)

        self.ackermann_pub.publish(self.ackermenn_msg)

def main(args=None):

    rclpy.init(args=args)

    ackermann_converter = AckermannConverter()
    rclpy.spin(ackermann_converter)
    ackermann_converter.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
