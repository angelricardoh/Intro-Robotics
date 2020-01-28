"""
Sample Code for Lab2 for testing the servo motor
Use "run.py [--sim] lab2_servo_test" to execute
"""


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()

    def run(self):
        self.servo.go_to(90)
        self.time.sleep(2)
        self.servo.go_to(-90)
        self.time.sleep(2)
        self.servo.go_to(0)
        self.time.sleep(1)
