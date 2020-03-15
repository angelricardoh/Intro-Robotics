from pyCreate2 import create2
import math
import numpy as np
import odometry
import pd_controller2
import pid_controller
import matplotlib
from timeit import default_timer as timer


# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt

THRESHOLD = 0.01
PLOT_INTERVAL = 20


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(500, 40, 20, [0, 0], [-300, 300], is_angle=False)
        self.result = np.empty((0, 5))
        self.base_speed = 100

    def go_to_goal(self, goal_x, goal_y, threshold=THRESHOLD):
        # distance = utils.euclidean_distance(goal_x, goal_y)
        state = self.create.update()
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                #
                # new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta),
                #            self.odometry.x, self.odometry.y]
                # self.result = np.vstack([self.result, new_row])

                # base:
                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                self.create.drive_direct(int(self.base_speed+output_theta), int(self.base_speed-output_theta))
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                if distance < THRESHOLD:
                    break



    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        waypoints = [
            [0.32, 1.11],
            [0.0, 1.72],
            [1.88, 2.50],
            [1.82, 1.19],
            [3.32, 1.19]
        ]

        # waypoints = [
        #     [2, 0],
        #     [2, 1],
        #     [0, 1],
        #     [0, 0]
        # ]

        start = timer()
        for x,y in waypoints:
            self.go_to_goal(x, y)

        end = timer()
        print("total elapsed time " + str(end - start) + " seg")

        plt.figure()
        plt.plot(self.result[:, 3], self.result[:, 4])
        plt.savefig("position.png")
