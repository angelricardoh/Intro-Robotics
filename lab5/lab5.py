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

BASE_SPEED = 0


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pdTheta = pd_controller.PDController(75, 1, -200, 200)
        # self.pdTheta = pd_controller.PDController(200, 10, -75, 75)
        # self.pdTheta = pd_controller.PDController(500, 100, -200, 200)
        self.pidTheta = pid_controller.PIDController(1000, 20, 100, -200, 200, -200, 200)
        # self.pidTheta = pid_controller.PIDController(500, 100, 1000, -200, 200, -500, 500)
        # self.pidTheta = pid_controller.PIDController(1500, 50, 100, -200, 200, -500, 500)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal_theta = math.pi / 2
        # goal_x = ...
        # goal_y = ...
        base_speed = 0

        result = np.empty((0, 3))
        end_time = self.time.time() + 10
        print("goal = ", math.degrees(goal_theta))
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print(self.odometry.theta)
                print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta)]
                result = np.vstack([result, new_row])

                reference = float(goal_theta)
                measured = float(self.odometry.theta)
                # reference = math.degrees(goal_theta)
                # measured = math.degrees(self.odometry.theta)

                # Section 2.2
                # delta_power = self.pdTheta.update(reference, measured, self.time.time())
                delta_power = self.pidTheta.update(reference, measured, self.time.time())

                self.create.drive_direct(int(BASE_SPEED + delta_power), int(BASE_SPEED - delta_power))

        # plotting for go-to-angle goal_theta:
        plt.title("Angle")
        plt.plot(result[:, 0], result[:, 1], label="odometry")
        plt.plot(result[:, 0], result[:, 2], label="goal")
        plt.grid()
        plt.legend()
        plt.savefig("lab6_angle.png")  # make s ure to not overwrite plots

        # plotting for go-to-goal (goal_x, goal_x):
        # plt.figure()
        # f, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        # ax1.set_title("Angle")
        # ax1.plot(result[:,0], result[:,1], label="odometry")
        # ax1.plot(result[:,0], result[:,2], label="goal")
        # ax1.grid()
        # ax1.legend()

        # ax2.set_title("Position")
        # ax2.plot(result[:,3], result[:,4], label="odometry")
        # ax2.scatter([goal_x], [goal_y], color="r", s=40, label="goal")
        # ax2.axis("equal")
        # ax2.grid()
        # ax2.legend()
        # plt.savefig("lab6_position.png")
