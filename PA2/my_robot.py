import math
from pyCreate2 import create2
from enum import Enum


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
        self.l1 = 0.4  # dist(joint2, joint4), the lower arm length
        self.l2 = 0.39  # dist(joint4, joint6), the upper arm length
        self.arm_height_offset = 0.3105
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def joint_go_to_angle(self, joint_number, angle_in_radians):
        self.arm.go_to(joint_number, angle_in_radians)

    def arm_go_to_angle(self, theta_1, theta_2):
        self.arm.go_to(Links.LOWER_ORANGE_LINK.value, theta_1)
        self.arm.go_to(Links.UPPER_ORANGE_LINK.value, theta_2)

        # self.z = self.l1 * math.cos(theta_1) + self.l2 * math.cos(theta_1 + theta_2)
        # self.x = self.l1 * math.sin(theta_1) + self.l2 * math.sin(theta_1 + theta_2)
        # self.x *= -1
        self.x = self.l1 * math.cos(theta_1) + self.l2 * math.cos(theta_1 + theta_2)
        self.y = self.l1 * math.sin(theta_1) + self.l2 * math.sin(theta_1 + theta_2)

        print("Go to % d, % d deg, FK: [% .2f, % .2f, % .2f]" \
              % (math.degrees(theta_1), math.degrees(theta_2), self.x, self.y, self.z))
        # print("Go to % d, % d deg, FK: [% .2f, % .2f, % .2f]" \
        #       % (math.degrees(theta_1), math.degrees(theta_2), self.x, self.y, self.z + self.arm_height_offset))

    def stop(self):
        self.create.drive_direct(0, 0)
        self.time.sleep(1000)
