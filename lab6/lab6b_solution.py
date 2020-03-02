from pyCreate2 import create2
import math
import numpy as np
import odometry
import pd_controller
import pd_controller2
import pid_controller
import matplotlib
from timeit import default_timer as timer


# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WAYPOINT_THRESHOLD = 0.05
WALL_THRESHOLD = 0.7
PLOT_INTERVAL = 20


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pd_controller = pd_controller.PDController(1000, 100, -75, 75)
        self.pd_controller = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pdTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pdDistance = pd_controller2.PDController(1000, 0, -300, 300, is_angle=False)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-300, 300], is_angle=False)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-300, 300], is_angle=False)
        self.pidWallFollow = pid_controller.PIDController(1000, 0, 100, [-75, 75], [-300, 300], is_angle=False)
        self.result = np.empty((0, 5))
        self.base_speed = 100

    def go_to_goal(self, goal_x, goal_y, threshold=WAYPOINT_THRESHOLD):
        print("go_to_goal x:" + str(self.odometry.x) + "y:" + str(self.odometry.y))

        skip_plot = 0
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))

            if skip_plot % PLOT_INTERVAL == 0:
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta),
                           self.odometry.x, self.odometry.y]
                self.result = np.vstack([self.result, new_row])
            else:
                skip_plot += 1

            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

    # def go_to_angle(self, goal_theta, threshold=THRESHOLD):
    #     while abs(self.odometry.theta - goal_theta)  > threshold:
    #         state = self.create.update()
    #         if state is not None:
    #             self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
    #             output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
    #             self.create.drive_direct(int(output_theta), int(-output_theta))

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

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5, is_get_dist: bool = False,
                    interrupt=lambda x: False):
        self.servo.go_to(angle)
        return self.sleep(sleep_time, is_get_dist=is_get_dist, interrupt=interrupt)

    def wall_following(self):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_distance = WALL_THRESHOLD

            distance = self.sonar.get_distance()
            if distance is not None:
                # print("wall following = " + str(distance))
                #output = self.p_controller.update(distance, goal_distance)
                output = self.pidWallFollow.update(distance, goal_distance, self.time.time())
                print("wall following " + str(output))
                self.create.drive_direct(int(self.base_speed - output), int(self.base_speed + output))
                self.time.sleep(0.01)
            end = timer()

    def follow_wall(self, dist_to_wall: float, goal_dist_to_wall: float, base_speed: float = 100.0) -> None:
        state = self.create.update()
        dist_to_wall = self.sonar.get_distance()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

            output_wall_follow = self.pidWallFollow.update(dist_to_wall, goal_dist_to_wall, self.time.time())
            print(output_wall_follow)

            v_right = int(base_speed - output_wall_follow)
            v_left = int(base_speed + output_wall_follow)
            # print("fw [v_right: %.2f, v_left: %.2f]" % (v_right, v_left))
            self.create.drive_direct(v_right, v_left)
            self.time.sleep(0.01)

    # def wall_following(self):
    #     goal_distance = WALL_THRESHOLD
    #
    #     distance = self.sonar.get_distance()
    #     # print(distance)
    #     if distance is not None:
    #         print("wall_following" + str(distance))
    #         output = self.pd_controller.update(distance, goal_distance, self.time.time())
    #         self.create.drive_direct(int(self.base_speed - output), int(self.base_speed + output))
    #         self.time.sleep(0.01)

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

        self.base_speed = 50
        while len(waypoints) > 0:
            print("new iteration")
            current_waypoint = waypoints.pop(0)
            goal_x = current_waypoint[0]
            goal_y = current_waypoint[1]
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            state = self.create.update()
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            prev_angle = 0
            current = "fwd"

            while distance > WAYPOINT_THRESHOLD:
                sonar_distance = self.sonar.get_distance()
                object_detected = True if sonar_distance < WALL_THRESHOLD else False
                if object_detected:
                    start = timer()
                    end = timer()
                    sonar_distance = self.sonar.get_distance()
                    while end - start < 1:
                        self.follow_wall(sonar_distance, WALL_THRESHOLD)
                        end = timer()
                    state = self.create.update()
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    curr_angle = math.degrees(self.odometry.theta)
                    turn_angle = - (curr_angle - prev_angle)
                    prev_angle = curr_angle
                    # turn_angle = goal_theta + curr_angle
                    print("turn angle = ", str(turn_angle))
                    self.go_to_angle(turn_angle, 1)
                    print("object_detected")
                else:
                    self.go_to_angle(0, 1)
                    self.go_to_goal(goal_x, goal_y)
                state = self.create.update()
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))



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
