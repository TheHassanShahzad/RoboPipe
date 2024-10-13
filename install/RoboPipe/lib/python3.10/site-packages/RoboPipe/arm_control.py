import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pybullet as p
import pybullet_data

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        self.subscription = self.create_subscription(
            Point,
            'target_positions',  # Corrected topic name
            self.move_arm,
            10)
        self.subscription  # prevent unused variable warning

        # Connect to the physics server
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path for the data files

        # Load the plane
        planeId = p.loadURDF("plane.urdf")

        # Load the Kuka robot arm
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf", startPos, startOrientation)

        # Enable gravity
        p.setGravity(0, 0, -9.81)

        # Define the IK solver parameters
        self.ikSolver = p.IK_DLS  # You can use other solvers like p.IK_LMA or p.IK_SDLS
        self.maxIter = 1000  # Maximum iterations
        self.threshold = 1e-4  # Threshold for convergence

        # Get the indices of the end effector link
        self.endEffectorIndex = 6  # Adjust according to your robot's configuration

    def move_arm(self, msg):
        targetPos = [msg.x, msg.y, msg.z]
        
        # Perform inverse kinematics to move the end effector to the target position
        jointPoses = p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, targetPos, solver=self.ikSolver,
                                                maxNumIterations=self.maxIter, residualThreshold=self.threshold)

        # Set the joint positions to move the robot arm
        for i in range(len(jointPoses)):
            p.setJointMotorControl2(self.robotId, i, p.POSITION_CONTROL, jointPoses[i])
        p.stepSimulation()

def main(args=None):
    rclpy.init(args=args)
    arm_control = ArmControl()
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
