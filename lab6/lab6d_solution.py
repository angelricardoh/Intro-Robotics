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

WAYPOINT_THRESHOLD = 0.005
WALL_THRESHOLD = 0.7
PLOT_INTERVAL = 20


class State(Enum):
    init = 0,
    go_to_goal = 1,
    init_wall_following = 2,
    wall_following = 3,
    finished = 4


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pd_controller = pd_controller.PDController(1000, 100, -75, 75)
        # self.pd_controller = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # self.pdTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        # self.pdDistance = pd_controller2.PDController(1000, 0, -300, 300, is_angle=False)
        self.pidTheta = pid_controller.PIDController(1000, 5, 50, [-10, 10], [-300, 300], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-300, 300], is_angle=False)
        self.pidWallFollow = pid_controller.PIDController(300, 0, 100, [-75, 75], [-300, 300], is_angle=False)
        self.result = np.empty((0, 5))
        self.base_speed = 300

    def sleep(self, time_in_sec, is_get_dist: bool = False, interrupt=lambda x: False):
        result = math.inf
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            t = self.time.time()

            if is_get_dist:
                result = min(self.sonar.get_distance(), result)

            if start + time_in_sec <= t or interrupt(result):
                break

        return None if not is_get_dist else result

    def get_dist_to_goal(self, goal_x: float, goal_y: float) -> float:
        return math.sqrt((goal_x - self.odometry.x) ** 2 + (goal_y - self.odometry.y) ** 2)

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5, is_get_dist: bool = False,
                    interrupt=lambda x: False):
        self.servo.go_to(angle)
        return self.sleep(sleep_time, is_get_dist=is_get_dist, interrupt=interrupt)

    def sweep_sonar(self, turn_angle, curr_angle: float = None, sleep_time: float = 1.0,
                    interrupt=lambda x: False) -> float:
        if curr_angle is None:
            curr_angle = math.degrees(self.odometry.theta)

        min_dist_to_wall = math.inf

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        dist = self.go_to_angle(curr_angle - turn_angle, sleep_time, is_get_dist=True, interrupt=interrupt)
        if interrupt(dist):
            return dist
        min_dist_to_wall = min(min_dist_to_wall, dist, self.sonar.get_distance())

        self.servo.go_to(curr_angle)
        self.sleep(sleep_time)

        return min_dist_to_wall

    def go_to_goal(self, goal_x: float, goal_y: float) -> None:
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

            dist_to_goal = self.get_dist_to_goal(goal_x, goal_y)
            output_distance = self.pidDistance.update(0, dist_to_goal, self.time.time())

            v_right = int(output_theta + output_distance)
            v_left = int(-output_theta + output_distance)
            # print("gtg [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
            self.create.drive_direct(v_right, v_left)

    def follow_wall(self, dist_to_wall: float, goal_dist_to_wall: float, base_speed: float = 100.0) -> None:
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            output_wall_follow = self.pidWallFollow.update(dist_to_wall, goal_dist_to_wall, self.time.time())
            print("follow_wall" + str(output_wall_follow))

            v_right = int(base_speed - output_wall_follow)
            v_left = int(base_speed + output_wall_follow)
            # print("fw [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
            self.create.drive_direct(v_right, v_left)

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

        dist_threshold = 0.01
        # wall_threshold = 0.4 + self.odometry.w / 2
        # goal_dist_to_wall = wall_threshold + 0.1
        wall_threshold = 0.7
        goal_dist_to_wall = wall_threshold - 0.1
        wall_follow_timeout = 2.0

        self.base_speed = 100
        while len(waypoints) > 0:
            print("new iteration")
            current_waypoint = waypoints.pop(0)
            goal_x = current_waypoint[0]
            goal_y = current_waypoint[1]
            curr_state = State.go_to_goal

            print("-----------------\nGoing to @{%.4f, %.4f}" % (goal_x, goal_y))
            prev_angle = math.degrees(self.odometry.theta)
            while self.get_dist_to_goal(goal_x, goal_y) > dist_threshold:
                dist_to_wall = self.sonar.get_distance()
                curr_angle = math.degrees(self.odometry.theta)
                # curr_robot_angle = (((curr_angle + 90) % 180) - 90)
                # dist_to_wall = self.sweep_sonar(sonar_sweep_angle, sonar_sweep_sleep_time, curr_robot_angle)
                # print("dist_to_wall: %.4f" % dist_to_wall)
                print("dist_to_goal: %.4f" % self.get_dist_to_goal(goal_x, goal_y))
                # self.go_to_angle(curr_sonar_angle, sonar_sweep_sleep_time)
                while (dist_to_wall is not None and dist_to_wall > wall_threshold
                       and curr_state is not State.finished):
                    if self.get_dist_to_goal(goal_x, goal_y) <= dist_threshold:
                        curr_state = State.finished
                        break

                    self.go_to_angle(0, 0.01)

                    curr_state = State.go_to_goal
                    self.go_to_goal(goal_x, goal_y)
                    dist_to_wall = self.sonar.get_distance()

                if curr_state is not State.init:
                    prev_angle = math.degrees(self.odometry.theta)
                while (dist_to_wall is not None and dist_to_wall <= wall_threshold
                       and curr_state is not State.finished):
                    if self.get_dist_to_goal(goal_x, goal_y) <= dist_threshold:
                        curr_state = State.finished
                        break

                    curr_state = State.wall_following

                    self.follow_wall(dist_to_wall, goal_dist_to_wall, base_speed=100)

                    curr_angle = math.degrees(self.odometry.theta)
                    print("fw [dist_to_wall: %.4f]\nfw [curr_angle: %.4f]\n" % (dist_to_wall, curr_angle))

                    # turn_angle = -(((curr_angle + 90) % 180) - 90)
                    turn_angle = -(curr_angle - prev_angle)
                    self.go_to_angle(turn_angle, 0.1)
                    dist_to_wall = self.sonar.get_distance()

                if curr_state is State.wall_following:
                    self.sleep(wall_follow_timeout)
                    curr_state = State.init


        # start = timer()
        # for x,y in waypoints:
        #     if waypoints[len(waypoints) - 1] == (x,y):
        #         self.go_to_goal(x, y, threshold=0.02)
        #     self.go_to_goal(x, y)
        #
        # end = timer()
        # print("total elapsed time " + str(end - start) + " seg")

        plt.figure()
        plt.plot(self.result[:, 3], self.result[:, 4])
        plt.savefig("position.png")
