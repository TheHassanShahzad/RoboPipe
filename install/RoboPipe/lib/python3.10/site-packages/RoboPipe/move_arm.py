#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import time
from example_interfaces.msg import String

def StringPrinter(msg):
    print(msg.data)
    
def main(args=None):
# Connect to the physics server
    rclpy.init(args=args) 
    node = Node("load_arm")
    node.create_subscription(String, "robot_news", StringPrinter, 10)
    
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path for the data files

    # Load the plane
    planeId = p.loadURDF("plane.urdf")

    # Load the Kuka robot arm
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("kuka_iiwa/model.urdf", startPos, startOrientation, )

    # Enable gravity
    p.setGravity(0, 0, -9.81)

    # Simulation loop
    for _ in range(10000):
        p.stepSimulation()
        time.sleep(0.01)
    


if __name__ == "__main__":
    main()

