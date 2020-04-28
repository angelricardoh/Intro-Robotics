import operator
from enum import Enum
import math
import pyCreate2
import numpy as np


class Joints(Enum):
    BASE = 0
    LOWER_ORANGE = 1
    MIDDLE_ORANGE = 2
    UPPER_ORANGE = 3
    SPHERE = 4
    GRIPPER = 5


SPEED_FACTOR = 2.0
SLEEP_FACTOR = 0.05
CUP_Z_COORDINATE = 0.12  # Z_AXIS
INITIAL_PICKUP_ANGLE = -70

ADJACENT_SIDE_TO_SHELVES = 0.5  # X_AXIS
OPPOSITE_SIDE_TO_SHELVES = 0.6  # Y_AXIS
ANGLE_TO_WALL = math.atan2(OPPOSITE_SIDE_TO_SHELVES, ADJACENT_SIDE_TO_SHELVES)


class MyArmRobot:
    def __init__(self, arm, time):
        self.arm = arm
        self.time = time
        self.lower_joint_angle = 0
        self.upper_joint_angle = 0
        self.sphere_angle = 0
        self.gripper_angle = 0

        self.distance_cup_arm_x_axis = None
        self.distance_cup_arm_y_axis = None
        self.hypotenuse_distance_cup = None
        self.angle_to_cup = None

    def set_create_coordinates(self, coor_x, coor_y):
        print("Coordinates")
        print(coor_x)
        print(coor_y)
        self.hypotenuse_distance_cup = math.sqrt(coor_x * coor_x + \
                                                 coor_y * coor_y)
        self.angle_to_cup = math.atan2(coor_x, coor_y)

    def grab_cup_and_go_to_initial_position(self):
        print("Initial params")
        print(self.hypotenuse_distance_cup)
        print(self.angle_to_cup)
        print(math.degrees(self.angle_to_cup))


        self.arm.open_gripper()
        self.time.sleep(4)

        self.go_to(Joints.GRIPPER, -90)
        self.time.sleep(1)

        print("rotate base")

        self.go_to(Joints.BASE, -math.degrees(self.angle_to_cup))
        self.time.sleep(3)

        print("inverse kinematics")

        self.inverse_kinematics(-(self.hypotenuse_distance_cup) + 0.327, CUP_Z_COORDINATE)
        self.time.sleep(3)

        print("goto GRIPPER")

        self.go_to(Joints.GRIPPER, -65)
        self.time.sleep(3)

        print("close GRIPPER")


        self.arm.close_gripper()
        self.time.sleep(4)

        self.go_to(Joints.SPHERE, 0)
        self.time.sleep(1)

        self.move_body(65, 45)

        self.move_gripper_effectors(None, -20)
        self.move_body(40, 0)

        self.move_gripper_effectors(None, 45)
        self.move_body(20, 0)

        self.move_gripper_effectors(-180, 90)
        self.move_body(0, 0)

        self.go_to(Joints.BASE, 0)
        self.time.sleep(3)

    def place_cup_in_shelf_zero(self):
        self.go_to(Joints.BASE, 90)
        self.time.sleep(3)

        self.move_gripper_effectors(180, 0)
        self.inverse_kinematics(0.785, 0.375, smooth=True)

        self.move_gripper_effectors(None, 22.5)

        for angle_degrees in np.arange(90, math.degrees(ANGLE_TO_WALL) + 5, -1):
            print(angle_degrees)
            self.go_to(Joints.BASE, angle_degrees)
            self.time.sleep(0.01)

        self.arm.open_gripper()
        self.time.sleep(4)

        self.go_to(Joints.BASE, 90)
        self.move_body(0, 0)
        self.time.sleep(100)

        pass

    def place_cup_in_shelf_one(self):
        self.move_gripper_effectors(-90, None)
        self.go_to(Joints.BASE, 90 - math.degrees(ANGLE_TO_WALL))
        self.time.sleep(1)
        self.inverse_kinematics(0.54, 0.675, smooth=True)
        self.move_gripper_effectors(-180, None)

        self.arm.open_gripper()
        self.time.sleep(4)

        self.move_gripper_effectors(None, self.gripper_angle - 15)
        self.move_gripper_effectors(-90, None)

        self.move_body(0, None)
        self.move_body(0, 0)

        self.time.sleep(100)
        self.leave_cup()

    def place_cup_in_shelf_two(self):
        self.move_gripper_effectors(-90, None)
        self.inverse_kinematics(OPPOSITE_SIDE_TO_SHELVES - 0.17, 0.80, smooth=True)
        self.move_gripper_effectors(-180, None)

        self.leave_cup()

    def place_cup_in_shelf_three(self):
        self.go_to(Joints.GRIPPER, 75)
        self.time.sleep(1)

        self.inverse_kinematics(OPPOSITE_SIDE_TO_SHELVES - 0.334, 1.0534)
        self.time.sleep(1)

        self.leave_cup()

    def leave_cup(self):
        self.arm.open_gripper()
        self.time.sleep(4)

        self.move_body(0, 0)
        self.time.sleep(100)

    def forward_kinematics(self, theta1, theta2):
        self.go_to(Joints.LOWER_ORANGE, theta1)
        self.go_to(Joints.UPPER_ORANGE, theta2)
        print("Go to {},{} deg".format(theta1, theta2))

    def inverse_kinematics(self, x_i, z_i, smooth=False, calculate_angle=False):
        L1 = 0.4  # estimated using V-REP (joint2 - joint4)
        L2 = 0.39  # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        x = -x_i
        # compute inverse kinematics
        r = math.sqrt(x * x + z * z)
        try:
            alpha = math.acos((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))
            theta2 = math.pi - alpha

            beta = math.acos((r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))
            theta1 = math.atan2(x, z) - beta
            if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
                theta2 = math.pi + alpha
                theta1 = math.atan2(x, z) + beta
            if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
                print("Not possible")
                return False
        except:
            return False
        if calculate_angle:
            return True

        if smooth:
            self.move_body(math.degrees(theta1), math.degrees(theta2))
        else:
            self.go_to(Joints.LOWER_ORANGE, math.degrees(theta1))
            self.go_to(Joints.UPPER_ORANGE, math.degrees(theta2))

        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

    def __move(self, to_theta_1, to_theta_2, body=None):
        if body:
            from_theta_1 = self.lower_joint_angle
            from_theta_2 = self.upper_joint_angle
        else:
            from_theta_1 = self.sphere_angle
            from_theta_2 = self.gripper_angle
        factor_theta_1 = 1
        factor_theta_2 = 1
        abs_diff_theta_1 = abs(to_theta_1 - from_theta_1)
        abs_diff_theta_2 = abs(to_theta_2 - from_theta_2)

        if abs_diff_theta_1 == 0:
            factor_theta_1 = 0
        elif abs_diff_theta_2 == 0:
            factor_theta_2 = 0
        elif abs_diff_theta_1 > abs_diff_theta_2:
            factor_theta_2 = abs_diff_theta_2 / abs_diff_theta_1
        else:
            factor_theta_1 = abs_diff_theta_1 / abs_diff_theta_2

        theta_1_increase = operator.add if to_theta_1 > from_theta_1 else operator.sub
        theta_2_increase = operator.add if to_theta_2 > from_theta_2 else operator.sub

        theta_1 = from_theta_1
        theta_2 = from_theta_2

        while not (theta_1 == to_theta_1 and theta_2 == to_theta_2):
            if body:
                self.forward_kinematics(theta_1, theta_2)
            else:
                print("Move sphere and gripper to {},{} deg".format(theta_1, theta_2))
                self.go_to(Joints.SPHERE, theta_1)
                self.go_to(Joints.GRIPPER, theta_2)
            self.time.sleep(SLEEP_FACTOR)

            theta_1 = theta_1_increase(theta_1, factor_theta_1 * SPEED_FACTOR)
            theta_2 = theta_2_increase(theta_2, factor_theta_2 * SPEED_FACTOR)

            if theta_1_increase == operator.add and theta_1 > to_theta_1:
                theta_1 = to_theta_1
            if theta_1_increase == operator.sub and theta_1 < to_theta_1:
                theta_1 = to_theta_1

            if theta_2_increase == operator.add and theta_2 > to_theta_2:
                theta_2 = to_theta_2
            if theta_2_increase == operator.sub and theta_2 < to_theta_2:
                theta_2 = to_theta_2

    def move_body(self, to_theta_1=None, to_theta_2=None):
        if to_theta_1 is None:
            to_theta_1 = self.lower_joint_angle
        if to_theta_2 is None:
            to_theta_2 = self.upper_joint_angle

        self.__move(to_theta_1, to_theta_2, body=True)

        self.forward_kinematics(to_theta_1, to_theta_2)
        self.time.sleep(SLEEP_FACTOR)

    def move_gripper_effectors(self, to_theta_1=None, to_theta_2=None):
        if to_theta_1 is None:
            to_theta_1 = self.sphere_angle
        if to_theta_2 is None:
            to_theta_2 = self.gripper_angle

        self.__move(to_theta_1, to_theta_2, body=False)

        self.go_to(Joints.SPHERE, to_theta_1)
        self.go_to(Joints.GRIPPER, to_theta_2)
        self.time.sleep(SLEEP_FACTOR)

    def print_arm_reachability(self):
        # Test IK range
        for x in np.arange(0.0, 1.20, 0.005):
            opposite = math.sin(ANGLE_TO_WALL) * x
            print("opposite = %.3f, x = %.3f:" % (opposite, x), end='')
            for z in np.arange(0.0, 1.20, 0.005):
                # TODO: Reuse test_inverse_kinematics
                if self.inverse_kinematics(x, z, None, True):
                    print("z %.3f:" % z, end=' ')
                self.time.sleep(0.001)
            print()

    def go_to(self, joint: Joints, degrees, smooth=False):
        if joint == Joints.LOWER_ORANGE:
            self.lower_joint_angle = degrees
        elif joint == Joints.UPPER_ORANGE:
            self.upper_joint_angle = degrees
        elif joint == Joints.SPHERE:
            self.sphere_angle = degrees
        elif joint == Joints.GRIPPER:
            self.gripper_angle = degrees

        self.arm.go_to(joint.value, math.radians(degrees))
