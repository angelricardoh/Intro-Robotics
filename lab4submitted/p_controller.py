class PController:
    def __init__(self, kp, minOutput, maxOutput):
        self.Kp = kp
        self.minOutput = minOutput
        self.maxOutput = maxOutput

    def update(self, currentState, desireState):
        error = desireState - currentState
        output = self.Kp * error
        return min(self.maxOutput, max(self.minOutput, output))

