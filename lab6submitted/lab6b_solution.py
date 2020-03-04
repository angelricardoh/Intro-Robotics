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

WAYPOINT_THRESHOLD = 0.01
WALL_THRESHOLD = 0.7
WALL_MAINTAIN_DISTANCE = WALL_THRESHOLD - 0.1


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
        self.pidTheta = pid_controller.PIDController(1000, 5, 50, [-10, 10], [-300, 300], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-300, 300], is_angle=False)
        self.pidWallFollowing = pid_controller.PIDController(300, 0, 100, [-75, 75], [-300, 300], is_angle=False)
        self.result = np.empty((0, 3))
        self.base_speed = 300

    def get_goal_distance(self, goal_x, goal_y):
        return math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

    def sleep(self, time_in_sec, is_get_dist: bool = False, interrupt=lambda x: False):
        result = math.inf
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.update_odometry(state)

            t = self.time.time()

            if is_get_dist:
                result = min(self.sonar.get_distance(), result)

            if start + time_in_sec <= t or interrupt(result):
                break

        return None if not is_get_dist else result

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5, is_get_dist: bool = False,
                    interrupt=lambda x: False):
        self.servo.go_to(angle)
        return self.sleep(sleep_time, is_get_dist=is_get_dist, interrupt=interrupt)

    def go_to_goal(self, goal_x: float, goal_y: float) -> None:
        state = self.create.update()
        if state is not None:
            self.update_odometry(state)

            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

            output_distance = self.pidDistance.update(0, self.get_goal_distance(goal_x, goal_y), self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

    def wall_following(self, distance_to_wall, base_speed: float = 100.0) -> None:
        state = self.create.update()
        # distance_to_wall = self.sonar.get_distance()
        if state is not None:
            self.update_odometry(state)

            output_wall_follow = self.pidWallFollowing.update(distance_to_wall, WALL_MAINTAIN_DISTANCE,
                                                              self.time.time())
            # print("wall_following" + str(output_wall_follow))

            v_right = int(base_speed - output_wall_follow)
            v_left = int(base_speed + output_wall_follow)
            # print("fw [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
            self.create.drive_direct(v_right, v_left)

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

        wall_follow_timeout = 2.0

        start = timer()

        self.base_speed = 100
        state = "goal"
        while len(waypoints) > 0:
            current_goal = waypoints.pop(0)
            goal_x = current_goal[0]
            goal_y = current_goal[1]
            current_state = "goal"

            # print("-----------------\nGoing to @{%.4f, %.4f}" % (goal_x, goal_y))
            previous_angle = math.degrees(self.odometry.theta)
            while self.get_goal_distance(goal_x, goal_y) > WAYPOINT_THRESHOLD:
                distance_to_wall = self.sonar.get_distance()
                current_angle = math.degrees(self.odometry.theta)
                # print("dist_to_goal: %.4f" % self.get_goal_distance(goal_x, goal_y))
                while (distance_to_wall is not None and distance_to_wall > WALL_THRESHOLD
                       and current_state != "finished"):
                    if self.get_goal_distance(goal_x, goal_y) <= WAYPOINT_THRESHOLD:
                        current_state = "finished"
                        break

                    self.go_to_angle(0, 0.01)

                    current_state = "goal"
                    self.go_to_goal(goal_x, goal_y)
                    distance_to_wall = self.sonar.get_distance()

                if current_state is not "init":
                    previous_angle = math.degrees(self.odometry.theta)
                while (distance_to_wall is not None and distance_to_wall <= WALL_THRESHOLD
                       and current_state != "finished"):
                    if self.get_goal_distance(goal_x, goal_y) <= WAYPOINT_THRESHOLD:
                        current_state = "finished"
                        break

                    current_state = "wall_following"

                    self.wall_following(distance_to_wall, base_speed=100)

                    current_angle = math.degrees(self.odometry.theta)
                    # print("fw [distance_to_wall: %.4f]\nfw [curr_angle: %.4f]\n" % (distance_to_wall, current_angle))

                    turn_angle = - (current_angle - previous_angle)
                    self.go_to_angle(turn_angle, 0.1)
                    distance_to_wall = self.sonar.get_distance()

                if current_state == "wall_following":
                    self.sleep(wall_follow_timeout)
                    current_state = "init"

        end = timer()
        print("total elapsed time " + str(end - start) + " seg")

        plt.figure()
        plt.plot(self.result[:, 1], self.result[:, 2])
        plt.savefig("position.png")
