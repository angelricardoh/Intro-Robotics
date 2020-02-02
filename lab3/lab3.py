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
        self.my_robot = MyRobot(self.create, self.time)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # forward
        self.printState()
        self.my_robot.forward(1, 0.1)
        self.printState()

        # forward & backward
        self.my_robot.backward(1, 0.1)

        self.printState()
        # turn left
        self.my_robot.turn_left(1, 0.182)
        self.my_robot.forward(1, 0.1)

        self.printState()
        # turn right
        self.my_robot.turn_right(1, 0.182)
        self.my_robot.forward(1, 0.1)
        self.my_robot.turn_right(1, 0.182)
        self.printState()
        # drive and stop in 10 seg
        self.create.drive_direct(100, 100)
        self.my_robot.stop(10)
        self.printState()
        # drive and stop inmediately
        self.my_robot.turn_right(1, 0.182)
        self.create.drive_direct(100, 100)
        self.my_robot.stop()

       # while True:
       #     state = self.create.update()
       #     if state is not None:
       #         print(state.__dict__)

    def printState(self):
        state = self.create.update()
        if state is not None:
            print(state.__dict__)
