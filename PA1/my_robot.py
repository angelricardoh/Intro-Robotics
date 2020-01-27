ROTATE_90_DEGREES_CONST_MM_PER_SEC = 182


class MyRobot:
    def __init__(self, create, time):
        self.__create = create
        self.__time = time
        self.base_speed = 0.1

    def forward(self, distance, speed=None):
        if speed is None:
            speed = self.base_speed
        mm_over_seg_speed = speed * 1000
        time = distance / speed
        self.__create.drive_direct(mm_over_seg_speed, mm_over_seg_speed)
        self.__time.sleep(time)

    def backward(self, distance, speed=None):
        if speed is None:
            speed = self.base_speed
        mm_over_seg_speed = speed * 1000
        time = distance / speed
        self.__create.drive_direct(-mm_over_seg_speed, -mm_over_seg_speed)
        self.__time.sleep(time)

    def turn_left(self):
        self.forward(1, 0.1)
        self.turn_left_inplace()
        self.forward(1, 0.1)

    def turn_right(self):
        self.forward(1, 0.1)
        self.turn_right_inplace()
        self.forward(1, 0.1)

    def turn_left_inplace(self, duration=None, speed=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)
        # elif speed:
        #     duration = (ROTATE_90_DEGREES_CONST / 1000) / speed
        #     print("here")
        print(duration)
        print(speed)
        self.__create.drive_direct(speed, -speed)
        self.__time.sleep(duration)

    def turn_right_inplace(self, duration=None, speed=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)
        # elif speed:
        #     duration = (ROTATE_90_DEGREES_CONST / 1000) / speed
        #     print("here")
        self.__create.drive_direct(-speed, speed)
        self.__time.sleep(duration)

    def stop(self):
        self.__create.stop()
