import time
import math
import numpy as np

# if on the robot, don't use X backend
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

D = 0.072  # mts
W = 0.235  # mts
N = 508.8  # counts / revolutions


def get_delta(traveled_counts_encoder):
    # traveled_counts_encoder = traveled_counts_encoder%32768
    return (D * math.pi * traveled_counts_encoder) / N


class MyRobot:
    def __init__(self, create, time_helper, base_speed=None, position_tracking=False):
        self.__create = create
        self.__time_helper = time_helper
        self.base_speed = base_speed  # base_speed 100 mm/s
        self.x = 0.0
        self.y = 0.0
        self.theta = 0
        self.prev_r_count = 0
        self.prev_l_count = 0
        self.position_tracking = position_tracking
        self.count = 0
        self.result = np.empty((0, 4))
        self.first_ground_truth_lecture = True
        self.initial_ground_truth_x = 0
        self.initial_ground_truth_y = 0

        _ = self.__create.sim_get_position()

    def reset_count(self):
        state = self.__create.update()
        if state is not None:
            l_count = state.__dict__['leftEncoderCounts']
            r_count = state.__dict__['rightEncoderCounts']
            self.prev_r_count = r_count
            self.prev_l_count = l_count

    def forward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        duration = distance / (mm_over_seg_speed / 1e3)
        self.__create.drive_direct(int(mm_over_seg_speed), int(mm_over_seg_speed))
        self.sleep(duration)

    def backward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        duration = distance / (mm_over_seg_speed / 1e3)
        self.__create.drive_direct(int(-mm_over_seg_speed), int(-mm_over_seg_speed))
        self.sleep(duration)

    def turn_left(self, duration, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(int(mm_over_seg_speed), int(-mm_over_seg_speed))
        self.sleep(duration)

    def turn_right(self, duration, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(int(-mm_over_seg_speed), int(mm_over_seg_speed))
        self.sleep(duration)

    def turn_left_90_degrees_inplace(self, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        print(mm_over_seg_speed)
        self.__create.drive_direct(int(mm_over_seg_speed), int(-mm_over_seg_speed))
        current_theta = self.theta
        while abs(math.degrees(current_theta) - math.degrees(self.theta)) < 90:
            self.update_odometry()

    def turn_right_90_degrees_inplace(self, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        print(mm_over_seg_speed)
        self.__create.drive_direct(int(-mm_over_seg_speed), int(mm_over_seg_speed))
        current_theta = self.theta
        while abs(math.degrees(current_theta) - math.degrees(self.theta)) < 90:
            self.update_odometry()

    def sleep(self, duration):
        if not self.position_tracking:
            self.__time_helper.sleep(duration)
            return

        start_time = self.__time_helper.time()
        end_time = start_time
        while end_time - start_time < duration:
            self.update_odometry()
            end_time = self.__time_helper.time()

    def print_state(self):
        state = self.__create.update()
        if state is not None:
            print("x = %.4f y = %.4f degrees = %.4f" % (self.x, self.y, self.theta))
            print(state.__dict__)

    def update_odometry(self):
        state = self.__create.update()
        if state is not None:
            l_count = state.__dict__['leftEncoderCounts']
            r_count = state.__dict__['rightEncoderCounts']

            delta_l = get_delta(l_count - self.prev_l_count)
            delta_r = get_delta(r_count - self.prev_r_count)

            self.prev_r_count = r_count
            self.prev_l_count = l_count

            delta_d = (delta_r + delta_l) / 2
            delta_theta = (delta_r - delta_l) / W
            self.theta += delta_theta
            self.x += delta_d * math.cos(self.theta)
            self.y += delta_d * math.sin(self.theta)

            ground_truth = self.__create.sim_get_position()
            # print("groundTruth = " + str(ground_truth))

            if self.first_ground_truth_lecture:
                self.initial_ground_truth_x = ground_truth[0]
                self.initial_ground_truth_y = ground_truth[1]
                self.first_ground_truth_lecture = False
            new_row = [self.x, self.y, ground_truth[0] - self.initial_ground_truth_x, \
                       ground_truth[1] - self.initial_ground_truth_y]
            self.result = np.vstack([self.result, new_row])
            theta_degrees = math.degrees(self.theta)

    def stop(self, duration=None):
        if duration is not None:
            self.sleep(duration)
        self.__create.drive_direct(0, 0)

    def get_mm_over_seg_speed(self, speed):
        if speed is None:
            speed = self.base_speed
        return speed * 1e3

    def plot_path(self):
        plt.title("Path")

        plt.plot(self.result[:, 0], self.result[:, 1], label="odometry")
        plt.plot(self.result[:, 2], self.result[:, 3], label="ground_truth")

        plt.grid()
        plt.legend()
        plt.savefig("hw2_path.png")  # make s ure to not overwrite plots
