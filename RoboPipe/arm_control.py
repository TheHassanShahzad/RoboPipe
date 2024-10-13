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
            'target_positions',
            self.move_arm,
            10)
        self.subscription  # prevent unused variable warning

        # Connect to the physics server
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Hide the left and right tabs
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Load the plane
        planeId = p.loadURDF("plane.urdf")

        # Load the Kuka robot arm
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf", startPos, startOrientation)

        # Enable gravity
        p.setGravity(0, 0, -9.81)

        # Define the IK solver parameters
        self.ikSolver = p.IK_DLS
        self.maxIter = 1000
        self.threshold = 1e-4

        # Get the indices of the end effector link
        self.endEffectorIndex = 6

    def move_arm(self, msg):
        targetPos = [msg.x, msg.y, msg.z]
        jointPoses = p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, targetPos, solver=self.ikSolver,
                                                maxNumIterations=self.maxIter, residualThreshold=self.threshold)
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
