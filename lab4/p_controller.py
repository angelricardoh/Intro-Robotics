class PController:
    def __init__(self, kp: float = 0.0, range_min: float = -500.0, range_max: float = 500.0):
        self.range_min = range_min
        self.range_max = range_max

        self.kp = kp

    def update(self, error: float) -> float:
        output = error * self.kp
        return self.clamping(output)

    def clamping(self, power) -> float:
        max(min(value, range_max), range_min)
