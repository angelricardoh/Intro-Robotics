class PDController:
    def __init__(self, kp, kd, minOutput, maxOutput):
        self.Kp = kp
        self.Kd = kd
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.last_time = 0
        self.last_error = 0

    def update(self, currentState, desireState, curr_time):
        error = desireState - currentState
        delta_time = curr_time - self.last_time
        delta_error = error - self.last_error
        output = self.Kp * error + self.Kd * (delta_error / delta_time)
        self.last_time = curr_time
        self.last_error = error
        return min(self.maxOutput, max(self.minOutput, output))

