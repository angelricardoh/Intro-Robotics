import utils


class PController:
    def __init__(self, kp: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.kp = kp

    def update(self, error: float) -> float:
        power = error * self.kp
        return utils.clamping(self.range_min, self.range_max, power)
