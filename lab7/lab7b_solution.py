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

WALL_THRESHOLD = 0.75
THRESHOLD = 0.01
PLOT_INTERVAL = 20


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidWallFollowing = pid_controller.PIDController(200, 50, 0, [0,0], [-50, 50], is_angle=False)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.result = np.empty((0, 5))
        self.base_speed = 100
        self.current = "fwd"

    def go_to_goal(self, goal_x, goal_y, threshold=THRESHOLD):
        start = self.time.time()
        end = self.time.time()
        # while end - start < 1:
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))

            # base:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(self.base_speed+output_theta), int(self.base_speed-output_theta))
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < THRESHOLD:
                print(distance)
                self.current = "finished"

                # goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                # output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                #
                # goal_distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                # output_distance = self.pidDistance.update(0, goal_distance, self.time.time())
                # self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                #
                # if distance < THRESHOLD:
                #     print(distance)
                #     self.current = "finished"



            end = self.time.time()

    def wall_following(self, goal_x, goal_y):
        start = self.time.time()
        end = self.time.time()

        # goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
        # print("wf: goal_angle", math.degrees(goal_theta))

        # self.go_to_angle(math.degrees(goal_theta));
        turn_left = False
        while end - start < 1:
            # print("wall_following")
            distance = self.sonar.get_distance()
            if distance is not None:
                output = self.pidWallFollowing.update(distance, WALL_THRESHOLD, self.time.time())
                self.create.drive_direct(int(self.base_speed - output), int(self.base_speed + output))
                self.time.sleep(0.01)
                if self.base_speed - output < self.base_speed + output:
                    turn_left = False
                else:
                    turn_left = True
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            end = self.time.time()
        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
        if turn_left is not None:
            if turn_left:
                print("wf: goal_angle", math.degrees(goal_theta))
                self.go_to_angle(math.degrees(-goal_theta))
            else:
                self.go_to_angle(math.degrees(goal_theta))


    def go_to_angle(self, angle: float = 0):
        self.servo.go_to(angle)
        self.sleep(2);

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()
            if start + time_in_sec <= t:
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
            [2.0, 0.0],
            [3.0, 2.0],
            [2.5, 2.0],
            [0.0, 1.5],
            [0.0, 0.0]
        ]

        start = timer()
        for x,y in waypoints:
            # Persistent algorithm
            self.current = "go_to_goal"
            # self.go_to_goal(x, y)
            while self.current != "finished":
                sonar_distance = self.sonar.get_distance()
                print("sonar_distance = ", sonar_distance)
                if self.current == "wall_following" or sonar_distance < WALL_THRESHOLD:
                    self.wall_following(x, y)
                    self.current = "go_to_goal"
                    self.create.drive_direct(0, 0)
                    for offset in [-55, 0, 55]:
                        self.go_to_angle(offset)
                        distance = self.sonar.get_distance()
                        if distance < 0.75:
                            self.current = "wall_following"
                            break
                    self.go_to_angle(0);
                else:
                    # goal_theta = math.atan2(y - self.odometry.y, x - self.odometry.x)
                    #
                    # self.go_to_angle(math.degrees(goal_theta));
                    self.go_to_goal(x, y)


        end = timer()
        print("total elapsed time " + str(end - start) + " seg")

        plt.figure()
        plt.plot(self.result[:, 3], self.result[:, 4])
        plt.savefig("position.png")
