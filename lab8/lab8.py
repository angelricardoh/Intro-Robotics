import lab8_map
import math
import odometry
from pyCreate2 import create2
import pid_controller
import pd_controller
from particle_filter import ParticleFilter, Command

STRAIGHT_DISTANCE = 0.5

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
        self.particle_filter = ParticleFilter(
                                    map=self.map,
                                    virtual_create=self.virtual_create,
                                    num_particles=100,
                                    distance=STRAIGHT_DISTANCE,
                                    sigma_sensor=0.1,
                                    sigma_theta=0.05,
                                    sigma_distance=0.01,
                                )
        _ = self.create.sim_get_position()


    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi / 2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        # print(self.map.closest_distance((0.5, 0.5), 0))
        self.particle_filter.draw_particles()

        # This is an example on how to detect that a button was pressed in V-REP
        while True:
            # self.particle_filter.draw_particles()
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.forward(STRAIGHT_DISTANCE)
                self.particle_filter.movement(Command.straight, 0)
                # print("Forward pressed!")
            elif b == self.virtual_create.Button.TurnLeft:
                desired_angle = math.pi / 2
                # desired_angle %= 2 * math.pi
                self.particle_filter.movement(Command.turn_left, desired_angle)
                self.sleep(0.01)
                self.turn_left()

                # self.go_to_angle(self.odometry.theta+math.pi/2)
                # print("Turn Left pressed!")
            elif b == self.virtual_create.Button.TurnRight:
                desired_angle = -math.pi / 2
                # desired_angle %= 2 * math.pi
                self.particle_filter.movement(Command.turn_right, desired_angle)
                self.sleep(0.01)
                self.turn_right()

                # self.go_to_angle(self.odometry.theta-math.pi/2)
                # print("Turn Right pressed!")
            elif b == self.virtual_create.Button.Sense:
                distance = self.sonar.get_distance()
                self.particle_filter.sensing(distance)

            self.sleep(0.01)

    # def go_to_angle(self, goal_theta):
    #     while math.fabs(math.atan2(
    #             math.sin(goal_theta - self.odometry.theta),
    #             math.cos(goal_theta - self.odometry.theta))) > 0.01:
    #         output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
    #         self.create.drive_direct(+output_theta, -output_theta)
    #         self.update_odometry()
    #     self.create.drive_direct(0, 0)

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
