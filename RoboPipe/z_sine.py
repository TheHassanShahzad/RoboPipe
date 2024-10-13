import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import math

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('target_positions_publisher')

    publisher = node.create_publisher(Point, 'target_positions', 10)

    msg = Point()
    msg.x = 0.3
    msg.y = 0.4

    rate = node.create_rate(100)  # Adjust the rate as needed (e.g., 100 Hz)

    while True:
        msg.z = 0.3 + 0.2 * math.sin(time.time())  # Sine wave pattern for z coordinate
        publisher.publish(msg)
        node.get_logger().info('Publishing: X:%f, Y:%f, Z:%f' % (msg.x, msg.y, msg.z))
        #rate.sleep()
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
