#!/usr/bin/env python
import math


class PIDController:
    cumulative_error_range = 10

    def __init__(self, kp, kd, ki, min_output, max_output):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.minOutput = min_output
        self.maxOutput = max_output
        self.previousError = 0.0
        self.previousTime = None
        self.cumulative_error = [0] * self.cumulative_error_range
        self.error_index = 0

    def update(self, value, target_value, time):
        error = target_value - value
        self.cumulative_error[self.error_index] = error
        self.error_index += 1
        if self.error_index == self.cumulative_error_range:
            self.error_index = 0
        p = self.kp * error
        d, i = 0, 0
        if self.previousTime is not None:
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
                i = sum(self.cumulative_error)
        output = p + d + i
        self.previousTime = time
        self.previousError = error
        # print("error:", error)
        # print("output:", output)
        return max(min(output, self.maxOutput), self.minOutput)
