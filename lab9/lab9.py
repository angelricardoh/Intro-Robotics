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

        self.pf = ParticleFilter(origin, noise, 1000, self.map)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())

        self.min_dist_to_wall = self.travel_dist + 0.2
        self.min_dist_to_localize = 0.2
        self.min_theta_to_localize = math.pi / 4

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

        while not isLocalized:
            command = random.choice([c for c in Command])

            if command is Command.FORWARD:
                self.forward()
            elif command is Command.TURN_LEFT:
                self.turn_left()
            elif command is Command.TURN_RIGHT:
                self.turn_right()

            self.sense()
            self.time.sleep(0.01)

            dist_position_to_goal = math.sqrt(
                (self.odometry.x - self.filter.x) ** 2 + (self.odometry.y - self.filter.y) ** 2)
            diff_theta_to_goal = abs(self.odometry.theta - self.filter.theta)

            isLocalized = dist_position_to_goal < self.min_dist_to_localize and diff_theta_to_goal < self.min_theta_to_localize

    def forward(self):
        print("Forward pressed!")
        FORWARD_DISTANCE = 0.5
        self.robot.forward(FORWARD_DISTANCE)
        self.robot.stop()
        self.pf.movement(FORWARD_DISTANCE, 0)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

    def turn_left(self):
        print("Turn Left pressed!")
        self.robot.turn_left(1.8)
        self.robot.stop()
        self.pf.movement(0, math.pi / 2)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

    def turn_right(self):
        print("Turn Right pressed!")
        self.robot.turn_right(1.8)
        self.robot.stop()
        self.pf.movement(0, -math.pi / 2)
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

    def sense(self):
        print("Sense pressed!")
        self.pf.sensing(self.sonar.get_distance())
        self.virtual_create.set_point_cloud(self.pf.get_particle_list())
        self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)

    def positioning(self):
        for _ in range(21):
            self.robot.turn_left(0.3)
            self.robot.stop()
            self.pf.movement(0, math.pi / 12)
            self.virtual_create.set_point_cloud(self.pf.get_particle_list())
            self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
            self.pf.sensing(self.sonar.get_distance())
            self.virtual_create.set_point_cloud(self.pf.get_particle_list())
            self.virtual_create.set_pose((self.pf.xyz), self.pf.theta)
