"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math
from enum import Enum
import random
import numpy as np
from scipy.stats import norm


class Links(Enum):
    LOWER_ORANGE_LINK = 1
    MIDDLE_ORANGE_LINK = 2
    UPPER_ORANGE_LINK = 3
    BEHIND_SPHERE = 4
    SPHERE = 5


class MyRobot:
    def __init__(self, arm, time):
        self.arm = arm
        self.time = time
        self.l1 = 0.4
        self.l2 = 0.39
        self.arm_height_offset = 0.3105
        self.position = None
        self.angle = None

    def joint_go_to_angle(self, joint_number, angle_in_radians):
        self.arm.go_to(joint_number, angle_in_radians)

    def arm_go_to_angle(self, theta_1, theta_2):
        self.arm.go_to(Links.LOWER_ORANGE_LINK.value, theta_1)
        self.arm.go_to(Links.UPPER_ORANGE_LINK.value, theta_2)

        x = self.l1 * math.cos(theta_1) + self.l2 * math.cos(theta_1 + theta_2)
        y = self.l1 * math.sin(theta_1) + self.l2 * math.sin(theta_1 + theta_2)

        self.angle = "Go to % d, % d deg, FK: [%.2f, %.2f, %.2f]" \
              % (math.degrees(theta_1), math.degrees(theta_2), x, y + self.arm_height_offset, 0)

    def go_to_position(self, x, y):
        y -= self.arm_height_offset

        r = euclidean_distance(0, 0, x, y)

        clampped_alpha_parameters = clamp(((self.l1 ** 2 + self.l2 ** 2 - r ** 2) / (2 * self.l1 * self.l2)), -1, 1)
        clampped_beta_parameters = clamp(((r ** 2 + self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * r)), -1, 1)
        alpha = math.acos(clampped_alpha_parameters)
        beta = math.acos(clampped_beta_parameters)

        theta_1 = math.atan2(y, x) - abs(beta) - math.pi / 2
        theta_1_sol2 = math.atan2(y, x) + abs(beta) - math.pi / 2
        theta_2 = math.pi - abs(alpha)
        theta_2_sol2 = math.pi + abs(alpha)

        self.position = "Go to [%d, %d], IK: [%.2f deg, %.2f deg]" % (x, y, theta_1, theta_2)

        self.arm_go_to_angle(theta_1, theta_2)

    def draw_point(self, x, z, color=None, wait_time=3):
        if color is None:
            color = clamp(x, 0, 1), random.random(), clamp(z, 0, 1)

        self.go_to_position(x, z)
        self.arm.set_color(*color)
        self.time.sleep(wait_time)

    def draw_points(self, point_list, color=None, wait_time=2):
        self.arm.enable_painting()

        for point in point_list:
            self.draw_point(*point, color=color, wait_time=wait_time)

        self.arm.disable_painting()

    def draw_rectangle(self, start_x, start_z, end_x, end_z, step=0.1, color=None, wait_time=1):
        self.arm.enable_painting()
        draw_color = color

        for z in np.arange(0, end_z - start_z, step):
            if color is None:
                draw_color = random.random(), random.random(), random.random()

            for x in np.arange(0, end_x - start_x, step):
                if x == 0:
                    self.draw_point(start_x + x, start_z + z, color=draw_color)
                else:
                    self.draw_point(start_x + x, start_z + z, color=draw_color, wait_time=wait_time)

        self.arm.disable_painting()

    def draw_line(self, start_x, start_z, end_x, end_z, step=0.1, wait_time=1):

        self.arm.enable_painting()
        draw_color = random.random(), random.random(), random.random()

        z_array = None;
        x_array = None;
        if end_z - start_z == 0:
            z = np.arange(10, dtype=float)
            z_array = np.full_like(z, start_z)
        else:
            z_array = np.arange(start_z, end_z, (end_z - start_z) / 10)
        if end_x - start_x == 0:
            x = np.arange(10, dtype=float)
            x_array = np.full_like(x, start_x)
        else:

            x_array = np.arange(start_x, end_x, (end_x - start_x) / 10)
        for x, z in zip(x_array, z_array):
            self.draw_point(x, z, color=draw_color, wait_time=wait_time)

        self.arm.disable_painting()


def euclidean_distance(start_x, start_y, dest_x, dest_y):
    return math.sqrt((dest_x - start_x) ** 2 + (dest_y - start_y) ** 2)


def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)


def dist(p0, p1):
    return np.sqrt(np.sum((p1 - p0) ** 2))


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()
        self.my_robot = MyRobot(arm, self.time)

    def run(self):

        # Section 1

        self.my_robot.joint_go_to_angle(Links.SPHERE.value, math.pi / 2)
        self.my_robot.joint_go_to_angle(Links.BEHIND_SPHERE.value, - math.pi / 2)
        self.time.sleep(2)


        # Section 2 Forward Kinematics

        # print("Using joint 42 (range 0 to 90 degrees)")
        # for angle in range(0, 90):
        #     self.my_robot.arm_go_to_angle(42, math.radians(angle))
        #     print(self.my_robot.angle)

        # # Joint 88 out of bounds I guess joint 88 does not exist
        # print("Using joint 88 (range -180 to 0 degrees)")
        # # for angle in range(-180, 0):
        # #     self.my_robot.joint_go_to_angle(88, math.radians(angle))
        # #     print(self.my_robot.angle)

        # # Alternative use: self.my_robot.arm_go_to_angle(math.radians(45), math.radians(-90))
        # self.my_robot.arm_go_to_angle(math.pi / 4, -math.pi / 2)
        # print(self.my_robot.angle)
        # self.time.sleep(5)
        # self.my_robot.arm_go_to_angle(math.pi / 2, -7 * math.pi / 18)
        # print(self.my_robot.angle)
        # self.time.sleep(5)
        # self.my_robot.arm_go_to_angle(1 * math.pi / 9, -math.pi)
        # print(self.my_robot.angle)
        # self.time.sleep(5)
        # self.my_robot.arm_go_to_angle(math.pi / 4, -1 * math.pi / 18)
        # print(self.my_robot.angle)
        # self.time.sleep(5)


        # 3 Inverse Kinematics

        # self.my_robot.go_to_position(0, 0)
        # print(self.my_robot.position)
        # self.time.sleep(5)
        # self.my_robot.go_to_position(0, 1)
        # print(self.my_robot.position)
        # self.time.sleep(5)
        # self.my_robot.go_to_position(0, 2)
        # print(self.my_robot.position)
        # self.time.sleep(5)


        # 4.1 rectangle attempt 1

        # point_list = [(-0.3, 1), (0.3, 1), (0.3, 0.9), (-0.3, 0.9)]
        # self.my_robot.draw_points(point_list)


        # 4.2 rectangle attempt 2

        # self.my_robot.draw_rectangle(-0.3, 0.9, 0.3, 1, step=0.05, wait_time=1)


        # 4.3 custom path

        self.my_robot.go_to_position(0.0, 1.3)
        self.my_robot.draw_line(0.0, 1.3, -0.3, 1.0, step=0.05, wait_time=1)
        self.my_robot.draw_line(-0.3, 1.0, 0.0, 0.7, step=0.05, wait_time=1)
        self.my_robot.draw_line(0.0, 0.7, 0.3, 1.0, step=0.05, wait_time=1)
        self.my_robot.draw_line(0.3, 1.0, 0.0, 1.3, step=0.05, wait_time=1)


        self.time.sleep(5)
