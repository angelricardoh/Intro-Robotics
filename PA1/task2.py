ROTATE_90_DEGREES_CONST_MM_PER_SEC = 182

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

        self.forward(1, 0.1)
        self.turn_left_inplace(10, None)
        self.forward(1, 0.1)
        self.stop()

    def forward(self, distance, speed):
        mm_over_seg_speed = speed * 1000
        time = distance / speed
        self.create.drive_direct(mm_over_seg_speed, mm_over_seg_speed)
        self.time.sleep(time)

    def backward(self, distance, speed):
        mm_over_seg_speed = speed * 1000
        time = distance / speed
        self.create.drive_direct(-mm_over_seg_speed, -mm_over_seg_speed)
        self.time.sleep(time)

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
        self.create.drive_direct(speed, -speed)
        self.time.sleep(duration)

    def turn_right_inplace(self, duration=None, speed=None):
        if duration:
            speed = int(ROTATE_90_DEGREES_CONST_MM_PER_SEC / duration)
        # elif speed:
        #     duration = (ROTATE_90_DEGREES_CONST / 1000) / speed
        #     print("here")
        print(duration)
        print(speed)
        self.create.drive_direct(-speed, speed)
        self.time.sleep(duration)

    def stop(self):
        self.create.stop()
