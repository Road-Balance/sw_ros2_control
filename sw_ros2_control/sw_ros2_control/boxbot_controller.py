import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class


from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type
from geometry_msgs.msg import Twist # Enable use of the std_msgs/Float64MultiArray message type

# Twist Sub => Float64MultiArray Pub

class BoxBotController(Node):

    def __init__(self):
        super().__init__('boxbot_controller')

        self.twist_sub = self.create_subscription(Twist,
                                            '/cmd_vel',
                                            self.cmd_vel_callback,
                                            10)
        self.mul_arr_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.pub_msg = Float64MultiArray()

    def cmd_vel_callback(self, msg):

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.pub_msg.data = [linear_x]

        self.mul_arr_pub.publish(self.pub_msg)

def main(args=None):
 
  rclpy.init(args=args)
 
  boxbot_controller_node = BoxBotController()
  rclpy.spin(boxbot_controller_node)
  boxbot_controller_node.destroy_node()

  rclpy.shutdown()
 
if __name__ == '__main__':
  main()