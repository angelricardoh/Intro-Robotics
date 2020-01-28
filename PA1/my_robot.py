ROTATE_90_DEGREES_CONST_MM_PER_SEC = 182


class MyRobot:
    def __init__(self, create, time, base_speed=None):
        self.__create = create
        self.__time = time
        self.base_speed = 0.1  # base_speed 100 mm/s

    def forward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        time = distance / speed
        self.__create.drive_direct(mm_over_seg_speed, mm_over_seg_speed)
        self.__time.sleep(time)

    def backward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        time = distance / speed
        self.__create.drive_direct(-mm_over_seg_speed, -mm_over_seg_speed)
        self.__time.sleep(time)

    def turn_left_inplace(self, duration, speed):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(mm_over_seg_speed, -mm_over_seg_speed)
        self.__time.sleep(duration)

    def turn_right_inplace(self, duration, speed):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(-mm_over_seg_speed, mm_over_seg_speed)
        self.__time.sleep(duration)

    def turn_left_90_degrees_inplace(self, duration=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)

        self.__create.drive_direct(speed, -speed)
        self.__time.sleep(duration)

    def turn_right_90_degrees_inplace(self, duration=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)

        self.__create.drive_direct(-speed, speed)
        self.__time.sleep(duration)

    def turn_left(self):
        self.forward(1, 0.1)
        self.turn_left_inplace()
        self.forward(1, 0.1)

    def turn_right(self):
        self.forward(1, 0.1)
        self.turn_right_inplace()
        self.forward(1, 0.1)

    def stop(self):
        self.__create.stop()

    def wait(self, duration):
        self.__time.sleep(duration)
        self.__create.stop()

    def get_mm_over_seg_speed(self, speed):
        if speed is None:
            speed = self.base_speed
        return speed * 1e3



