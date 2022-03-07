import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class

from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type
from ackermann_msgs.msg import AckermannDriveStamped

# Twist Sub => Float64MultiArray Pub

class RacecarController(Node):

    def __init__(self):
        super().__init__('boxbot_controller')

        self.ackermann_sub = self.create_subscription(AckermannDriveStamped,
                                            '/ackermann_commands',
                                            self.ackermann_callback,
                                            10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.throttling_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        self.steering_msg = Float64MultiArray()
        self.throttling_msg = Float64MultiArray()

    def ackermann_callback(self, msg):

        throttle = msg.drive.speed / 0.1
        steer = msg.drive.steering_angle

        self.steering_msg.data = [steer, steer]
        self.throttling_msg.data = [throttle, throttle, throttle, throttle]
        
        print(throttle, steer)
        self.steering_pub.publish(self.steering_msg)
        self.throttling_pub.publish(self.throttling_msg)

def main(args=None):
 
  rclpy.init(args=args)
 
  racecar_controller = RacecarController()
  rclpy.spin(racecar_controller)
  racecar_controller.destroy_node()

  rclpy.shutdown()
 
if __name__ == '__main__':
  main()