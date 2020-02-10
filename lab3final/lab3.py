"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""
from my_robot import MyRobot
from pyCreate2 import create2


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

    def run(self):
        self.my_robot = MyRobot(self.create, self.time)
        self.my_robot.position_tracking = True
        
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        default_turn_speed = 0.182 # Default speed to make 90 degree turns in 10 seg
        self.my_robot.reset_count()
        self.my_robot.print_state()
        # Lab 3 section 2.1
        # while True:
        #     state = self.create.update()
        #     if state is not None:
        #         print(state.__dict__)

        # Lab 3 section 2.2
        # self.my_robot.forward(1)
        # self.my_robot.update_odometry()

        # self.my_robot.backward(1)
        # self.my_robot.update_odometry()

        # self.my_robot.turn_left_90_degrees_inplace(0.05)
        # self.my_robot.update_odometry()

        # # Lab 3 section 4.1 1-meter square clockwise
        # self.my_robot.forward(1)
        # self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        # self.my_robot.forward(1)
        # self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        # self.my_robot.forward(1)
        # self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        # self.my_robot.forward(1)

        # Lab 3 section 4.2 1-meter square counter-clockwise
        # self.my_robot.forward(1)
        #self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        #self.my_robot.forward(1)
        #self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        #self.my_robot.forward(1)
        #self.my_robot.turn_left_90_degrees_inplace(default_turn_speed)
        #self.my_robot.forward(1)

        # Lab 3 section 4.3 15-meter straight ine
        self.my_robot.forward(15,0.05)

        self.my_robot.stop()
