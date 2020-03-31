import math
from pyCreate2 import create2

class MyRobot:
    def __init__(self, create, time, base_speed):
        self.create = create
        self.time = time
        self.base_speed = base_speed

    def forward(self, distance):
        self.create.drive_direct(int(self.base_speed * 1000.0), int(self.base_speed * 1000.0))
        self.time.sleep(distance / self.base_speed)

    def backward(self, distance):
        self.create.drive_direct(int(-self.base_speed * 1000.0), int(-self.base_speed * 1000.0))
        self.time.sleep(distance / self.base_speed)

    def stop(self):
        self.create.drive_direct(0, 0)
        self.time.sleep(10000)

    def turn_left(self, duration):
        self.create.drive_direct(int(self.base_speed * 1000.0), int(-self.base_speed * 1000.0))
        self.time.sleep(duration)

    def turn_right(self, duration):
        self.create.drive_direct(int(-self.base_speed * 1000.0), int(self.base_speed * 1000.0))
        self.time.sleep(duration)
