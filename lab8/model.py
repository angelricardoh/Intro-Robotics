import lab8_map
import math
import odometry
from pyCreate2 import create2
import pid_controller
import pd_controller
from particle_filter import ParticleFilter, Command
import random

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
        self.pdTheta = pd_controller.PDController(500, 40, -200, 200)
        self.pidTheta = pid_controller.PIDController(200, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.map = lab8_map.Map("lab8_map.json")
        self.base_speed = 100
        self.odometry = odometry.Odometry()
        self.particle_filter = ParticleFilter()
        _ = self.create.sim_get_position()
        self.factory = factory

    def run(self):
        # self.create = self.factory.create_create()
        # self.virtual_create = self.factory.create_virtual_create()

        self.create.start()
        self.create.safe()

        self.create.start_stream([
                    create2.Sensor.LeftEncoderCounts,
                    create2.Sensor.RightEncoderCounts,
                ])

        sleepValue = random.randrange(1, 1000000) / 1000000
        sleepValue = 10 * sleepValue
        # print(sleepValue)
        self.sleep(sleepValue)
        self.virtual_create.set_pose((1.0, 0.5, 0.1), 0)

        self.forward(0.5)
        print("[{}]".format(self.odometry.x))




    def turn_left(self):
        self.create.drive_direct(self.base_speed, -self.base_speed)
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < 90:
            self.update_odometry()
        self.create.drive_direct(0, 0)

    def turn_right(self):
        self.create.drive_direct(int(-self.base_speed), int(self.base_speed))
        current_theta = self.odometry.theta
        while abs(math.degrees(current_theta) - math.degrees(self.odometry.theta)) < 90:
            self.update_odometry()
        self.create.drive_direct(0, 0)

    def forward(self, distance):
        self.create.drive_direct(int(self.base_speed), int(self.base_speed))
        self.sleep(distance / (self.base_speed / 1000))
        self.create.drive_direct(0, 0)

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

    def update_odometry(self):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
