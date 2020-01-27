"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""
from my_robot import MyRobot


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.my_robot = MyRobot(self.create, self.time)

    def run(self):
        self.create.start()
        self.create.safe()

        self.my_robot.base_speed = 0.1
        self.my_robot.forward(1, 0.1)
        self.my_robot.turn_left_inplace(1, None)
        self.my_robot.forward(1, 0.1)
        self.my_robot.stop()
