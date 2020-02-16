from p_controller import PController
from pd_controller import PDController

BASE_SPEED = 100


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        # define the gains here
        # self.kp = 100
        self.kp = 1000
        self.kd = 20
        self.minOutput = -250
        self.maxOutput = 250
        # instantiate your controllers here
        self.p_controller = PController(self.kp, self.minOutput, self.maxOutput)
        self.pd_controller = PDController(self.kp, self.kd, self.minOutput, self.maxOutput)

        self.previous_time_l = None
        self.previous_error_l = None

        self.previous_time_r = None
        self.previous_error_r = None

    def run(self):
        self.create.start()
        self.create.safe()

        self.servo.go_to(70)
        self.time.sleep(2)

        goal_distance = 0.5

        while True:
            distance = self.sonar.get_distance()
            if distance is not None:
                # update controllers and move robot here
                # ...

                differential_controller = True
                error = goal_distance - distance
                velocity_left_wheel = self.f_left(differential_controller, self.time.time(), error)
                velocity_right_wheel = self.f_right(differential_controller, self.time.time(), error)

                print("[curr_state = %f, error = %f, right: %f, left: %f]\n" % (
                    distance,
                    goal_distance - distance,
                    velocity_right_wheel,
                    velocity_left_wheel
                ))

                self.create.drive_direct(
                    int(velocity_right_wheel),
                    int(velocity_left_wheel)
                )

                self.time.sleep(0.1)

    def f_left(self, differential_controller, current_time, current_error) -> float:
        if differential_controller:
            delta_power = self.pd_controller.update(current_error, current_time, \
                                                   self.previous_error_l, self.previous_time_l)
        else:
            delta_power = self.p_controller.update(current_error)
        self.previous_time_l = current_time
        self.previous_error_l = current_error
        power = BASE_SPEED + delta_power
        return power

    def f_right(self, differential_controller, current_time, current_error) -> float:
        if differential_controller:
            delta_power = self.pd_controller.update(current_error, current_time, \
                                                   self.previous_error_r, self.previous_time_r)
        else:
            delta_power = self.p_controller.update(current_error)
        self.previous_time_r = current_time
        self.previous_error_r = current_error
        power = BASE_SPEED - delta_power
        return power
