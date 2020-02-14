from p_controller import PController
from pd_controller import PDController

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        # define the gains here
        self.kp = 1000
        self.kd = 3
        self.minOutput = -200
        self.maxOutput = 200
        # instantiate your controllers here
        self.p_controller = PController(self.kp, self.minOutput, self.maxOutput)
        self.pd_controller = PDController(self.kp, self.kd, self.minOutput, self.maxOutput)


    def run(self):
        self.create.start()
        self.create.safe()

        self.servo.go_to(70)
        self.time.sleep(2)

        goal_distance = 0.5
        base_speed = 100

        adjust = 1

        self.create.drive_direct(base_speed, base_speed)

        while True:
            distance = self.sonar.get_distance()
            if distance is not None:
                # print(distance)
                # update controllers and move robot here
                # ...
                u = self.p_controller.update(distance, goal_distance)
                u = self.pd_controller.update(distance, goal_distance, self.time.time())
                
                print(u)

                v_right = base_speed - u * adjust
                v_left = base_speed + u * adjust

                self.create.drive_direct(int(v_right), int(v_left))

                self.time.sleep(0.01)