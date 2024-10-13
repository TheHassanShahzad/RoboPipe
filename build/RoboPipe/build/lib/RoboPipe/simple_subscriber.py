import rclpy
from example_interfaces.msg import String

def callback(msg):
    print("Received message:", msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('simple_subscriber')

    subscriber = node.create_subscription(
        String,
        'robot_news',
        callback,
        10  # QoS profile depth
    )

    print("Listening for messages on 'robot_news'. Press Ctrl+C to exit.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
