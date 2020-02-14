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
        self.kp = 1700
        self.kd = 50
        self.minOutput = -250
        self.maxOutput = 250
        # instantiate your controllers here
        self.p_controller = PController(self.kp, self.minOutput, self.maxOutput)
        self.pd_controller = PDController(self.kp, self.kd, self.minOutput, self.maxOutput)

        self.current_time_l = None
        self.previous_time_l = None
        self.current_error_l = None
        self.previous_error_l = None

        self.current_time_r = None
        self.previous_time_r = None
        self.current_error_r = None
        self.previous_error_r = None

    def run(self):
        self.create.start()
        self.create.safe()

        self.servo.go_to(70)
        self.time.sleep(2)

        goal_distance = 0.5

        base_speed = BASE_SPEED
        v_left = BASE_SPEED
        v_right = BASE_SPEED

        while True:
            distance = self.sonar.get_distance()
            if distance is not None:
                # update controllers and move robot here
                # ...

                differential_controller = False
                v_left = int(self.f_left(differential_controller, v_left, base_speed, goal_distance - distance))
                v_right = int(self.f_right(differential_controller, v_right, base_speed, goal_distance - distance))

                print("[curr_state = %f, error = %f\nright: %f, left: %f]\n" % (
                    distance,
                    goal_distance - distance,
                    v_right,
                    v_left
                ))

                self.create.drive_direct(
                    v_right,
                    v_left
                )

                self.time.sleep(0.1)

    def f_left(self, differential_controller, base_speed, current_speed, u) -> float:
        self.current_time_l = self.time.time()
        self.current_error_l = u
        if u > 0:
            return base_speed
        if differential_controller:
            diff_power = self.pd_controller.update(self.current_error_l, self.current_time_l, \
                                                   self.previous_error_l, self.previous_time_l)
        else:
            diff_power = self.p_controller.update(u)
        self.previous_time_l = self.current_time_l
        self.previous_error_l = self.current_error_l
        power = (current_speed + diff_power)
        return power

    def f_right(self, differential_controller, base_speed, current_speed, u) -> float:
        self.current_time_r = self.time.time()
        self.current_error_r = u
        if u < 0:
            return base_speed
        if differential_controller:
            diff_power = self.pd_controller.update(self.current_error_r, self.current_time_r, \
                                                   self.previous_error_r, self.previous_time_r)
        else:
            diff_power = self.p_controller.update(u)
        self.previous_time_r = self.current_time_r
        self.previous_error_r = self.current_error_r
        power = (current_speed - diff_power)
        return power
