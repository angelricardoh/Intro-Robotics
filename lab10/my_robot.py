from typing import List, Callable
import sys


class MyRobot:
    def __init__(
            self,
            base_speed: float = None,  # base_speed in m/s
            drive_direct: Callable[[int, int], None] = lambda right_speed, left_speed: print(
                "Invalid drive_direct function", file=sys.stderr),
            sleep: Callable[[float], None] = lambda time: print(
                "Invalid sleep function", file=sys.stderr)
    ):
        self.base_speed = 0.1 if base_speed is None else base_speed
        self.drive_direct = drive_direct
        self.sleep = sleep

    def move(
            self,
            right_wheel_velocity_in_m_per_sec: float = None,
            left_wheel_velocity_in_m_per_sec: float = None,
            duration: float = None,  # in seconds
            distance: float = 1.0  # in meters
    ) -> None:
        def get_time(distance_: float, speed_: float) -> float:
            return abs(distance_ / speed_)

        if right_wheel_velocity_in_m_per_sec is None:
            right_wheel_velocity_in_m_per_sec = self.base_speed

        if left_wheel_velocity_in_m_per_sec is None:
            left_wheel_velocity_in_m_per_sec = self.base_speed

        if duration is None:
            duration = get_time(
                distance,
                average([right_wheel_velocity_in_m_per_sec, left_wheel_velocity_in_m_per_sec])
            )

        self.drive_direct(int(right_wheel_velocity_in_m_per_sec * 1000),
                          int(left_wheel_velocity_in_m_per_sec * 1000))
        self.sleep(duration)

    def stop(self) -> None:
        self.drive_direct(0, 0)

    def wait(self, duration: float = 1.0) -> None:
        self.stop()
        self.sleep(duration)

    def forward(self, distance: float = 1.0, speed: float = None) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(speed, speed, distance=distance)

    def backward(self, distance: float = 1.0, speed: float = None) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.forward(distance, -speed)

    def turn_left(self, duration: float = 1.0, speed: float = None) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(speed, -speed, duration)

    def turn_right(self, duration: float = 1.0, speed: float = None) -> None:  # speed in m/s
        if speed is None:
            speed = self.base_speed

        self.move(-speed, speed, duration)


def average(nums: List[float]) -> float:
    return sum(nums) / len(nums)
