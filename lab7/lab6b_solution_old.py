from pyCreate2 import create2
import math
import numpy as np
import odometry
import pd_controller
import pd_controller2
import pid_controller
import matplotlib
from timeit import default_timer as timer
from enum import Enum

# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WALL_THRESHOLD = 0.75


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pd_controller = pd_controller.PDController(1000, 100, -75, 75)
        # self.pd_controller = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # self.pdTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        # self.pdDistance = pd_controller2.PDController(1000, 0, -300, 300, is_angle=False)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # self.pidWallFollowing = pid_controller.PIDController(300, 0, 100, [-75, 75], [-300, 300], is_angle=False)
        self.pidWallFollowing = pid_controller.PIDController(200, 50, 0, [0, 0], [-50, 50])

        self.result = np.empty((0, 3))
        self.base_speed = 100
        self.current = ''

    def sleep(self, time_in_sec):
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5):
        self.servo.go_to(angle)
        return self.sleep(sleep_time)

    def go_to_goal(self, goal_x: float, goal_y: float) -> None:
        state = self.create.update()
        if state is not None:
            self.update_odometry(state)

            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(self.base_speed + output_theta), int(self.base_speed - output_theta))

    def wall_following(self) -> None:
        print("wf")
        start = self.time.time()
        end = self.time.time()

        # goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
        # print("wf: goal_angle", math.degrees(goal_theta))

        # self.go_to_angle(math.degrees(goal_theta));
        while end - start < 1:
            state = self.create.update()
            distance_to_wall = self.sonar.get_distance()

            print(end - start)
            if state is not None:
                self.update_odometry(state)

                output = self.pidWallFollowing.update(distance_to_wall, WALL_THRESHOLD, self.time.time())
                self.create.drive_direct(int(self.base_speed - output), int(self.base_speed + output))
                self.time.sleep(0.01)
            end = self.time.time()

    def update_odometry(self, state):
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        new_row = [self.time.time(),
                   self.odometry.x, self.odometry.y]
        self.result = np.vstack([self.result, new_row])

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

        WAYPOINT_THRESHOLD = 0.05
        wall_follow_timeout = 1.0

        start = timer()

        self.base_speed = 100
        self.current = "goal"
        while len(waypoints) > 0:
            current_goal = waypoints.pop(0)
            if len(waypoints) == 0:
                WAYPOINT_THRESHOLD = 0.01
            goal_x = current_goal[0]
            goal_y = current_goal[1]
            self.current = "goal"

            # print("-----------------\nGoing to @{%.4f, %.4f}" % (goal_x, goal_y))
            previous_angle = math.degrees(self.odometry.theta)
            goal_distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            self.go_to_angle(0, 1)
            current_x = self.odometry.x
            current_y = self.odometry.y
            while goal_distance > WAYPOINT_THRESHOLD:
                distance_to_wall = self.sonar.get_distance()
                current_angle = math.degrees(self.odometry.theta)
                # print("dist_to_goal: %.4f" % self.get_goal_distance(goal_x, goal_y))
                while (distance_to_wall is not None and distance_to_wall > WALL_THRESHOLD
                       and self.current != "finished"):
                    goal_distance = math.sqrt(
                        math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if goal_distance <= WAYPOINT_THRESHOLD:
                        self.current = "finished"
                        break

                    self.current = "goal"
                    self.go_to_goal(goal_x, goal_y)

                    delta_distance = math.sqrt(
                        math.pow(current_x - self.odometry.x, 2) + math.pow(current_y - self.odometry.y, 2))
                    if delta_distance > 0.2:
                        self.go_to_angle(0, 0.01)
                    distance_to_wall = self.sonar.get_distance()

                previous_angle = math.degrees(self.odometry.theta)
                while (distance_to_wall is not None and distance_to_wall <= WALL_THRESHOLD
                       and self.current != "finished"):
                    goal_distance = math.sqrt(
                        math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if goal_distance <= WAYPOINT_THRESHOLD:
                        self.current = "finished"
                        break

                    self.current = "wall_following"

                    self.wall_following()
                    current_x = self.odometry.x
                    current_y = self.odometry.y

                    current_angle = math.degrees(self.odometry.theta)
                    # print("fw [distance_to_wall: %.4f]\nfw [curr_angle: %.4f]\n" % (distance_to_wall, current_angle))

                    turn_angle = - (current_angle - previous_angle)
                    self.go_to_angle(turn_angle, 0.1)
                    distance_to_wall = self.sonar.get_distance()

                if self.current == "wall_following":
                    self.create.drive_direct(0, 0)
                    for angle in [-70, 0, 70]:
                        self.go_to_angle(angle, 0)
                        distance = self.sonar.get_distance()
                        if distance < WALL_THRESHOLD - 0.1:
                            self.current = "wall_following"
                            break

        end = timer()
        print("total elapsed time " + str(end - start) + " seg")

        plt.figure()
        plt.plot(self.result[:, 1], self.result[:, 2])
        plt.savefig("position.png")
