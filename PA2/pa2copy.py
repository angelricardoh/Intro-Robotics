"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""
import math
import random
import numpy as np
from scipy.stats import norm
from utils import clamp


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()

        self.L1 = 0.4  # dist(joint2, joint4), the lower arm length
        self.L2 = 0.39  # dist(joint4, joint6), the upper arm length
        self.lower_arm_joint_number = 2 - 1
        self.upper_arm_joint_number = 4 - 1
        self.arm_height_offset = 0.3105
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def go_to_angle(self, angle_lower_arm, angle_upper_arm, is_print_info=True):
        # angle: < 0 to the right, > 0 to the left
        theta1 = math.radians(angle_lower_arm)
        theta2 = math.radians(angle_upper_arm)

        self.arm.go_to(self.lower_arm_joint_number, math.radians(angle_lower_arm))
        self.arm.go_to(self.upper_arm_joint_number, math.radians(angle_upper_arm))

        # self.x = self.L1 * math.cos(theta1 + math.pi / 2) + self.L2 * math.cos(theta1 + theta2 + math.pi / 2)
        # self.z = self.L1 * math.sin(theta1 + math.pi / 2) + self.L2 * math.sin(theta1 + theta2 + math.pi / 2)
        self.z = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        self.x = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        self.x = -self.x

        if is_print_info:
            print("Go to {:.2f}, {:.2f} deg, FK: [{:.2f}, {:.2f}, {:.2f}]"
                  .format(angle_lower_arm, angle_upper_arm, self.x, self.y, self.z + self.arm_height_offset))

    def go_to_position(self, x, y, is_print_info=True):
        goal_x = x
        goal_y = y - self.arm_height_offset

        r = get_dist(0, 0, goal_x, goal_y)
        alpha = math.acos((self.L1 ** 2 + self.L2 ** 2 - r ** 2) / (2 * self.L1 * self.L2))
        beta = math.acos((r ** 2 + self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * r))

        theta2 = math.pi - abs(alpha)
        theta1 = math.atan2(goal_y, goal_x) - abs(beta) - math.pi / 2

        # angle: < 0 to the right, > 0 to the left
        angle_lower_arm = math.degrees(theta1)
        angle_upper_arm = math.degrees(theta2)

        if is_print_info:
            print("Go to [{:.2f}, {:.2f}], IK: [{:.2f} deg, {:.2f} deg]"
                  .format(x, y, angle_lower_arm, angle_upper_arm))

        self.go_to_angle(angle_lower_arm, angle_upper_arm, is_print_info=False)

    def draw_point(self, x, y, color=None, wait_time=3, is_print_info=False):
        if color is None:
            color = clamp(x, 0, 1), random.random(), clamp(y, 0, 1)

        if is_print_info:
            print("color = {}".format(color))

        self.go_to_position(x, y, is_print_info=is_print_info)
        self.arm.set_color(*color)
        self.time.sleep(wait_time)

    def draw_points(self, point_list, color=None, wait_time=2):
        self.arm.enable_painting()

        for point in point_list:
            self.draw_point(*point, color=color, wait_time=wait_time)

        self.arm.disable_painting()

    def draw_rectangle(self, start_x, start_y, end_x, end_y, step=0.1, color=None, wait_time=1):
        self.arm.enable_painting()
        draw_color = color

        for y in np.arange(0, end_y - start_y, step):
            if color is None:
                draw_color = random.random(), random.random(), random.random()

            for x in np.arange(0, end_x - start_x, step):
                if x == 0:
                    self.draw_point(start_x + x, start_y + y, color=draw_color)
                else:
                    self.draw_point(start_x + x, start_y + y, color=draw_color, wait_time=wait_time)

        self.arm.disable_painting()

    def draw_customized(self, wait_time=1):
        x = np.arange(-0.7, 0.7, 0.05)
        y = norm.pdf(x, 0, 0.7)
        point_list = np.transpose(np.array([x, y]))

        self.draw_points(point_list, wait_time=wait_time)

    def run(self):
        # angle = -90
        self.arm.go_to(5, math.radians(90))
        self.arm.go_to(4, math.radians(-90))
        self.time.sleep(1)

        # get range of joint
        # for i in range(119, 125):
        #     # self.go_to_angle(0, -i)
        #     # self.go_to_angle(0, i)
        #     self.go_to_angle(-i, 180)
        #     # self.go_to_angle(i, -180)
        #     self.time.sleep(5)

        # 2 Forward Kinematics
        # print("Using joint 24 (range -121 to 121 degree)")
        # print("Using joint 46 (range -121 to 121 degree)")
        #
        # self.go_to_angle(45, -90)
        # self.time.sleep(3)
        # self.go_to_angle(90, -70)
        # self.time.sleep(3)
        # self.go_to_angle(20, -180)
        # self.time.sleep(3)
        # self.go_to_angle(45, -10)
        # self.time.sleep(3)
        #
        # # 3 Inverse Kinematics
        # self.go_to_position(0.5, 0.5)
        # self.time.sleep(3)
        # self.go_to_position(0, 1)
        # self.time.sleep(3)
        # self.go_to_position(-0.7, 0.5)
        # self.time.sleep(3)

        # 4.1 rectangle attempt 1
        # point_list = [(-0.3, 1), (0.3, 1), (0.3, 0.9), (-0.3, 0.9)]
        # self.draw_points(point_list)

        # 4.2 rectangle attempt 1
        # self.draw_rectangle(-0.3, 0.9, 0.3, 1, step=0.05, wait_time=1)

        # 4.3 customized drawing
        # self.draw_customized(wait_time=1)

        self.time.sleep(15)


def get_dist(start_x, start_y, dest_x, dest_y):
    return math.sqrt((dest_x - start_x) ** 2 + (dest_y - start_y) ** 2)
