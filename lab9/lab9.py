import lab8_map
import math
from my_robot import MyRobot
from particle_filter import ParticleFilter
from pyCreate2 import create2
import odometry
import random
from enum import Enum

class Command(Enum):
    FORWARD = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab8_map.Map("lab8_map.json")
        self.robot = MyRobot(self.create, self.time, base_speed=0.1)
        self.odometry = odometry.Odometry()
        self.travel_dist = 0.4
        self.current_dist_to_wall = 0
        self.base_speed = 100


        self.min_dist_to_wall = self.travel_dist + 0.2
        self.min_dist_to_localize = 0.4
        self.min_theta_to_localize = math.pi / 4


    def run(self):
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)

        origin = (0.5, 0.5, 0.1, 0)  # partical origin
        noise = (0.01, 0.05, 0.1)  # sd for (distance, theta, sonar)

        self.odometry.x = origin[0]
        self.odometry.y = origin[1]

        self.pf = ParticleFilter(origin, noise, 5000, self.map)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        # print(self.map.closest_distance((0.5,0.5), 0))

        # self.positioning()
        # This is an example on how to detect that a button was pressed in V-REP
        isLocalized = False

        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

        # self.pf.movement(0, 0)
        # self.sense()

        self.pf.movement(0, math.pi / 2)
        self.turn_left(False)
        self.pf.movement(0, math.pi / 2)
        self.turn_left(False)
        # self.pf.movement(0.5, 0)
        self.forward()
        self.pf.movement(0, -math.pi / 2)
        self.turn_right(True)
        self.sense()


        # self.pf.movement(0, math.pi / 2)
        # self.turn_left(False)
        # self.pf.movement(0, math.pi / 2)
        # self.turn_left(False)
        # self.pf.movement(0, math.pi / 2)
        # self.forward()
        # self.pf.movement(0, math.pi / 2)
        # self.turn_right(True)
        # self.sense()

        for angle in range(90, 360, 90):
            # self.go_to_angle(angle, 2)
            # self.turn_left_degrees(90)
            print("angle, ", angle)
            print("math_radians, ", math.radians(angle))
            self.turn_left(True)
            self.sense()
            # self.pf.movement(0, math.radians(angle))
            # self.current_dist_to_wall = self.sonar.get_distance()
            # self.pf.sensing(self.current_dist_to_wall, True)
            # self.virtual_create.set_point_cloud(self.pf.get_particle_list())
            # self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

        self.go_to_angle(0, 1)

        self.sense()
        self.forward()


        self.turn_left_degrees(100)
        for angle in range(100, -80, -20):
            self.turn_right_degrees(20)
            self.sense()
            if self.isLocalized():
                return

        if self.isLocalized():
            return

        self.go_to_angle(0, 1)

        self.turn_left(80)
        # self.sense()
        if self.isLocalized():
            return
        self.forward()
        self.sense()

        if self.isLocalized():
            return
        self.forward()
        if self.isLocalized():
            return
        while not isLocalized:
            command = random.choice([c for c in Command])
            print(command)
            self.current_dist_to_wall = self.sonar.get_distance()
            if command is Command.FORWARD:
                for angle in range(-50, 50, 25):
                    self.go_to_angle(angle, 2)
                    self.current_dist_to_wall = self.sonar.get_distance()
                    if self.current_dist_to_wall < 0.75:
                        self.go_to_angle(0, 1)
                        break
                self.go_to_angle(0, 1)
                self.forward()
            elif command is Command.TURN_LEFT:
                self.turn_left()
                self.current_dist_to_wall = self.sonar.get_distance()
                if self.current_dist_to_wall < 0.75:
                    self.turn_right()
                    continue
            elif command is Command.TURN_RIGHT:
                self.turn_right()
                self.current_dist_to_wall = self.sonar.get_distance()
                if self.current_dist_to_wall < 0.75:
                    self.turn_right()
                    continue

            self.sense()
            self.time.sleep(0.01)

            self.pf.theta %= 2 * math.pi
            odometry_theta = self.odometry.theta % (2 * math.pi)
            dist_position_to_goal = math.sqrt(
                (self.odometry.x - self.pf.x) ** 2 + (self.odometry.y - self.pf.y) ** 2)
            print("self.odometry.x " + str(self.odometry.x) + " self.odometry.y " + str(self.odometry.y) +
                  " self.odometry.theta " + str(odometry_theta))
            print("self.pf.x " + str(self.pf.x) + " self.pf.y " + str(self.pf.y) +
                  " self.pf.theta " + str(self.pf.theta))



            diff_theta_to_goal = abs(odometry_theta - self.pf.theta)
            print("DISTANCE_pos ", dist_position_to_goal)
            print("DISTANCE_theta ", diff_theta_to_goal)

            isLocalized = dist_position_to_goal < self.min_dist_to_localize and diff_theta_to_goal < self.min_theta_to_localize

    def sleep(self, time_in_sec):
        start = self.time.time()
        while True:
            self.update_odometry()
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, angle: float = 0, sleep_time: float = 0.5):
        self.servo.go_to(angle)
        return self.sleep(sleep_time)

    def turn_left(self, move_particles=True):
        if move_particles:
            self.pf.movement(0, math.pi / 2)
            self.virtual_create.set_point_cloud(self.pf.get_particle_list())
            self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
        self.create.drive_direct(self.base_speed, -self.base_speed)
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < 90:
            self.update_odometry()
        self.create.drive_direct(0, 0)

    def turn_right(self, move_particles=True):
        if move_particles:
            self.pf.movement(0, -math.pi / 2)
            self.virtual_create.set_point_cloud(self.pf.get_particle_list())
            self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
        self.create.drive_direct(int(-self.base_speed), int(self.base_speed))
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < 90:
            self.update_odometry()
        self.create.drive_direct(0, 0)


    def turn_left_degrees(self, degrees):
        self.pf.movement(0, math.radians(degrees))
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
        self.create.drive_direct(self.base_speed, -self.base_speed)
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < degrees:
            self.update_odometry()
        self.create.drive_direct(0, 0)

    def turn_right_degrees(self, degrees):
        self.pf.movement(0, -math.radians(degrees))
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
        self.create.drive_direct(-self.base_speed, self.base_speed)
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < degrees:
            self.update_odometry()
        self.create.drive_direct(0, 0)


    def forward(self):
        FORWARD_DISTANCE = 0.5
        self.pf.movement(FORWARD_DISTANCE, 0)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
        self.create.drive_direct(int(self.base_speed), int(self.base_speed))
        self.sleep(FORWARD_DISTANCE / (self.base_speed / 1000))
        self.create.drive_direct(0, 0)

    def sense(self, replace=True):
        print("Sense pressed!")
        self.current_dist_to_wall = self.sonar.get_distance()
        self.pf.sensing(self.current_dist_to_wall, replace)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)


    def isThetaLocalizaed(self):
        odometry_theta = self.odometry.theta % (2 * math.pi)


    def isLocalized(self):
        odometry_theta = self.odometry.theta % (2 * math.pi)
        dist_position_to_goal = math.sqrt(
            (self.odometry.x - self.pf.x) ** 2 + (self.odometry.y - self.pf.y) ** 2)
        print("self.odometry.x " + str(self.odometry.x) + " self.odometry.y " + str(self.odometry.y) +
              " self.odometry.theta " + str(odometry_theta))
        print("self.pf.x " + str(self.pf.x) + " self.pf.y " + str(self.pf.y) +
              " self.pf.theta " + str(self.pf.theta))

        diff_theta_to_goal = abs(odometry_theta - self.pf.theta)
        print("DISTANCE_pos ", dist_position_to_goal)
        print("DISTANCE_theta ", diff_theta_to_goal)

        return dist_position_to_goal < self.min_dist_to_localize and diff_theta_to_goal < self.min_theta_to_localize

    def update_odometry(self):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
