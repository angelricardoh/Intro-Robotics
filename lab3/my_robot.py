import time

ROTATE_90_DEGREES_CONST_MM_PER_SEC = 182

class MyRobot:
    def __init__(self, create, time, base_speed=None):
        self.__create = create
        self.__time = time
        self.base_speed = 0.1  # base_speed 100 mm/s

    def forward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        time = distance / (mm_over_seg_speed / 1e3)
        self.__create.drive_direct(mm_over_seg_speed, mm_over_seg_speed)
        self.sleep_track(time)

    def backward(self, distance, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        time = distance / (mm_over_seg_speed / 1e3)
        self.__create.drive_direct(-mm_over_seg_speed, -mm_over_seg_speed)
        self.sleep_track(time)

    def turn_left(self, duration, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(mm_over_seg_speed, -mm_over_seg_speed)
        self.sleep_track(duration)

    def turn_right(self, duration, speed=None):
        mm_over_seg_speed = self.get_mm_over_seg_speed(speed)
        self.__create.drive_direct(-mm_over_seg_speed, mm_over_seg_speed)
        self.sleep_track(duration)

    def turn_left_90_degrees_inplace(self, duration=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)

        self.__create.drive_direct(speed, -speed)
        self.sleep_track(duration)

    def turn_right_90_degrees_inplace(self, duration=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)

        self.__create.drive_direct(-speed, speed)
        self.sleep_track(duration)
        
    def sleep_track(self, duration):
        start_time = self.__time.time()
        end_time = start_time
        while end_time - start_time < duration:
            state = self.__create.update()
            if state is not None:
                print(state.__dict__)
            end_time = self.__time.time()

    def stop(self, duration=None):
        if duration is not None:
            self.sleep_track(duration)
        self.__create.drive_direct(0, 0)

    def get_mm_over_seg_speed(self, speed):
        if speed is None:
            speed = self.base_speed
        return speed * 1e3



