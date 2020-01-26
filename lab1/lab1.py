"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""
ROTATE_90_DEGREES_CONST = 19


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

    def run(self):
        self.create.start()
        self.create.safe()

        self.drive_straight()

        # self.drive_forward()

        # self.turn_right()

        # self.turn_left()

        self.rotate_backwards()

        self.sleep_drive_straight()

        self.create.stop()

    def drive_forward(self):
        self.create.drive_direct(0, 0)

    def turn_left(self):
        self.create.drive_direct(ROTATE_90_DEGREES_CONST, -ROTATE_90_DEGREES_CONST)

    def turn_right(self):
        self.create.drive_direct(-ROTATE_90_DEGREES_CONST, ROTATE_90_DEGREES_CONST)

    def rotate_backwards(self):
        self.create.drive_direct(-ROTATE_90_DEGREES_CONST * 2, ROTATE_90_DEGREES_CONST * 2)

    def drive_straight(self):
        self.create.drive_direct(100, 100)
        self.time.sleep(10)

    def sleep_drive_straight(self):
        self.time.sleep(10)
        self.drive_straight()
