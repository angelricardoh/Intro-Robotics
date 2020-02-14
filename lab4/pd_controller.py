class PDController:
    def __init__(self, kp: float = 0.0, kd: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.kp = kp
        self.kd = kd

    def update(self, error: float, time: float, previous_error: float = None, previous_time: float = None) -> float:
        if previous_error is None or previous_time is None or previous_time == 0:
            return error * self.kp
        time_diff = time - previous_time
        error_diff = error - previous_error
        derivative = error_diff / time_diff
        return error * self.kp + derivative * self.kd