from pyCreate2 import create2
import math
import numpy as np
import copy

# if on the robot, don't use X backend
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

import odometry
import pd_controller
import pid_controller
import utils

BASE_SPEED = 150

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pdTheta = pd_controller.PDController(500, 100, -200, 200)
        self.pidTheta = pid_controller.PIDController(500, 20, 1, -100, 100, -100, 100)
        self.pidTheta_dist = pid_controller.PIDController(500, 20, 1, -100, 100, -100, 100)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal_x = -0.5
        goal_y = -0.5
        distance = euclidean_distance(goal_x, goal_y)
        goal_theta = math.atan2(goal_x, goal_y)
        threshold = 0.01

        desired_angles = np.array([])
        measured_angles = np.array([])

        result = np.empty((0, 5))
        end_time = self.time.time() + 10

        print("Initial goal_distance " + str(distance))
        print("goal_theta = ", math.degrees(goal_theta))

        # End when threshold is reached
        while distance > threshold:
            state = self.create.update()
            if state is not None:

        # End after 10 seconds
        # while self.time.time() < end_time:
        #     state = self.create.update()
        #     if state is not None:

                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                # goal_theta %= 2 * np.pi
                print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta), self.odometry.x, self.odometry.y]
                result = np.vstack([result, new_row])

                distance = euclidean_distance(goal_x - self.odometry.x, goal_y - self.odometry.y)
                reference = float(goal_theta)
                measured = float(self.odometry.theta)

                desired_angles = np.append(desired_angles, reference)
                measured_angles = np.append(measured_angles, measured)

                # Section 4.1
                # # delta_power = self.pdTheta.update(reference, measured, self.time.time())
                # delta_power = self.pidTheta.update(reference, measured, self.time.time())
                # self.create.drive_direct(int(BASE_SPEED + delta_power), int(BASE_SPEED - delta_power))

                # Debugging
                # print("(" + str(self.odometry.x), "," + str(self.odometry.y) + ")" + "distance to goal ", str(distance))
                # print("goal_angle = " + str(goal_theta))
                # print("angel = " + str(self.odometry.theta))

                # Section 4.2
                # delta_power = self.pidTheta.update(reference, measured, self.time.time())
                # self.create.drive_direct(int(BASE_SPEED * distance + delta_power), int(BASE_SPEED * distance - delta_power))

                # Section 4.3
                # print(delta_power_dist)
                # print(delta_power)
                delta_power = self.pidTheta.update(reference, measured, self.time.time())
                delta_power_dist = self.pidTheta_dist.update(distance, 0, self.time.time())
                self.create.drive_direct(delta_power_dist + delta_power, delta_power_dist - delta_power)

        # plotting for go-to-goal (goal_x, goal_x):
        plt.figure()
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        ax1.set_title("Angle")
        ax1.plot(result[:, 0], result[:, 1], label="odometry")
        ax1.plot(result[:, 0], result[:, 2], label="goal")
        ax1.grid()
        ax1.legend()

        ax2.set_title("Position")
        ax2.plot(result[:, 3], result[:, 4], label="odometry")
        ax2.scatter([goal_x], [goal_y], color="r", s=40, label="goal")
        ax2.axis("equal")
        ax2.grid()
        ax2.legend()
        plt.savefig("lab5_position.png")

        np.savetxt("desired_angle_output.csv", desired_angles, delimiter=",")
        np.savetxt("measured_angle_output.csv", measured_angles, delimiter=",")


def euclidean_distance(x, y):
    return math.sqrt(pow(x, 2) + pow(y, 2))
