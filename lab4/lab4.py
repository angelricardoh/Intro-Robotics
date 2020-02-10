class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        # define the gains here
        self.kp = ...
        self.kd = ...
        self.minOutput = ...
        self.maxOutput = ...
        # instantiate your controllers here
        self.p_controller = PController(self.kp, self.minOutput, self.maxOutput)
        # self.pd_controller = PDController(self.kp, self.kd, self.minOutput, self.maxOutput)


    def run(self):
        self.create.start()
        self.create.safe()

        self.servo.go_to(70)
        self.time.sleep(2)

        goal_distance = 0.5
        base_speed = 100

        while True:
            distance = self.sonar.get_distance()
            if distance is not None:
                print(distance)
                # update controllers and move robot here
                # ...

                self.time.sleep(0.01)