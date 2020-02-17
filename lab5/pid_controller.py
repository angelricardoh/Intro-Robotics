import utils


class PIDController:
    def __init__(self, kp, kd, ki, range_min, range_max, acc_error_range_min, acc_error_range_max):
        self.range_min = range_min
        self.range_max = range_max
        self.min_integral_error = acc_error_range_min
        self.max_integral_error = acc_error_range_max

        self.previous_error = 0.0
        self.previous_time = None

        self.accumulated_error = 0.0

        self.kp = kp
        self.kd = kd
        self.ki = ki

    def update(self, reference, measured, time: float) -> float:
        error = reference - measured

        if self.previous_error is None or self.previous_time is None:
            self.update_error_time(error, time)
            return error * self.kp

        dt = time - self.previous_time
        de = error - self.previous_error
        self.accumulated_error += error
        self.accumulated_error = utils.clamping(self.min_integral_error, self.max_integral_error, self.accumulated_error)

        # Update error and time
        self.update_error_time(error, time)

        derivative = de / dt
        power = error * self.kp + derivative * self.kd + self.accumulated_error * dt
        return utils.clamping(self.range_min, self.range_max, power)

    def update_error_time(self, error, time):
        self.accumulated_error = error
        self.previous_error = error
        self.previous_time = time