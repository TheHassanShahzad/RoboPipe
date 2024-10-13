import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('random_positions_publisher')

    publisher = node.create_publisher(Point, 'target_positions', 10)

    msg = Point()

    rate = node.create_rate(10)  # Adjust the rate as needed (e.g., 10 Hz)

    while True:
        
        msg.x = random.uniform(-0.3, 0.3)
        msg.y = random.uniform(-0.3, 0.3)
        msg.z = random.uniform(-0.3, 0.3)
        
        publisher.publish(msg)
        node.get_logger().info('Publishing: X:%f, Y:%f, Z:%f' % (msg.x, msg.y, msg.z))
        time.sleep(1)
        #rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
