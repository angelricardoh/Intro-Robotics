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
        self.pidTheta = pid_controller.PIDController(75, 20, 1, -100, 100, -100, 100)
        # self.pidTheta = pid_controller.PIDController(500, 100, 50, -200, 200, -200, 200)
        # self.pidTheta = pid_controller.PIDController(1000, 20, 100, -200, 200, -200, 200)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        desired_angles = np.array([])
        measured_angles = np.array([])

        goal_theta = math.pi / 2
        # goal_theta = -math.pi / 2
        # goal_theta = math.pi

        result = np.empty((0, 3))
        end_time = self.time.time() + 10
        print("goal = ", math.degrees(goal_theta))
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta)]
                result = np.vstack([result, new_row])

                reference = float(goal_theta)
                measured = float(self.odometry.theta)

                desired_angles = np.append(desired_angles, reference)
                measured_angles = np.append(measured_angles, measured)

                # Section 2.2
                # delta_power = self.pdTheta.update(reference, measured, self.time.time())

                # Section 3.2
                delta_power = self.pidTheta.update(reference, measured, self.time.time())

                self.create.drive_direct(int(BASE_SPEED + delta_power), int(BASE_SPEED - delta_power))

        # plotting for go-to-angle goal_theta:
        plt.title("Angle")
        plt.plot(result[:, 0], result[:, 1], label="odometry")
        plt.plot(result[:, 0], result[:, 2], label="goal")
        plt.grid()
        plt.legend()
        plt.savefig("lab5_angle.png")  # make s ure to not overwrite plots

        np.savetxt("desired_angle_output.csv", desired_angles, delimiter=",")
        np.savetxt("measured_angle_output.csv", measured_angles, delimiter=",")
