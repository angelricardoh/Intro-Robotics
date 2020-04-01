"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math
from my_robot import MyRobot
from my_robot import Links


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()
        self.my_robot = MyRobot(self.arm, self.time)

    def run(self):
        # Section 1
        # self.my_robot.joint_go_to_angle(Links.SPHERE.value, math.pi / 2)
        # self.my_robot.joint_go_to_angle(Links.BEHIND_SPHERE.value, - math.pi / 2)
        # self.time.sleep(10)

        # Section 2
        print("Using joint 42 (range 0 to 90 degrees)")
        # for angle in range(0, 90):
        #     self.my_robot.arm_go_to_angle(42, math.radians(angle))

        print("Using joint 88 (range -180 to 0 degrees)")
        # for angle in range:
        #     self.my_robot.joint_go_to_angle(88, math.radians(angle))

        # self.my_robot.arm_go_to_angle(math.pi / 4, -math.pi / 2)
        # self.time.sleep(3)
        # self.my_robot.arm_go_to_angle(math.pi / 2, -7 * math.pi / 18)
        # self.time.sleep(3)
        # self.my_robot.arm_go_to_angle(1 * math.pi / 9, -math.pi)
        # self.time.sleep(3)
        # self.my_robot.arm_go_to_angle(math.pi / 4, -1 * math.pi / 18)
        # self.time.sleep(3)

        # 3 Inverse Kinematics
        self.my_robot.go_to_position(0, 0)
        self.time.sleep(3)
        self.my_robot.go_to_position(0, 1)
        self.time.sleep(3)
        self.my_robot.go_to_position(0, 2)
        self.time.sleep(3)

