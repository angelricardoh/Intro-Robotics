from pyCreate2 import create2
import math
import numpy as np

# if on the robot, don't use X backend
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import odometry
import pd_controller
import pid_controller


class Run:
    theta_kp = 500
    theta_kd = 40
    theta_ki = 20

    xy_kp = 1000
    xy_kd = 180
    xy_ki = 70

    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pdTheta = pd_controller.PDController(self.theta_kp, self.theta_kd, -200, 200)
        # self.pidTheta = pid_controller.PIDController(self.theta_kp, self.theta_kd, self.theta_ki, -200, 200)
        self.pidXY = pid_controller.PIDController(self.xy_kp, self.xy_kd, self.xy_ki, -200, 200)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal = "theta"

        goal_theta = [math.pi, math.pi/2, -math.pi/2][2]

        # goal_theta = math.pi / 4 

        goal_x = 0.5
        goal_y = 0.5

        if goal == "theta":
            result = np.empty((0,3))
            end_time = self.time.time() + 10
            while self.time.time() < end_time:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta)]
                    result = np.vstack([result, new_row])

                    # TODO call controller's update function 
                    # and the self.create.drive_direct(left, right) here
                    time = self.time.time()
                    # u = self.pidTheta.update(self.odometry.theta, goal_theta, time)
                    u = self.pdTheta.update(self.odometry.theta, goal_theta, time)
                    # print("feedback:", u)
                    self.create.drive_direct(int(u), int(-u))


            # plotting for go-to-angle goal_theta:
            plt.title("Angle")
            plt.plot(result[:,0], result[:,1], label="odometry")
            plt.plot(result[:,0], result[:,2], label="goal")
            plt.grid()
            plt.legend()
            plt.savefig("Robot_pd_angle_-90.png") # make sure to not overwrite plots

        else:
            result = np.empty((0,5))
            end_time = self.time.time() + 10
            while self.time.time() < end_time:
                state = self.create.update()
                if state is not None:
                    goal_theta = math.atan2((goal_y - self.odometry.y), (goal_x - self.odometry.x))
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta), self.odometry.x, self.odometry.y]
                    result = np.vstack([result, new_row])

                    # TODO call controller's update function 
                    # and the self.create.drive_direct(left, right) here
                    time = self.time.time()

                    current_displacement = math.sqrt(self.odometry.x ** 2 + self.odometry.y ** 2)
                    goal_displacement = math.sqrt(goal_x ** 2 + goal_y ** 2)

                    # Improved Solution 1
                    # Epsilon = 0.005
                    # if abs(goal_displacement - current_displacement) < Epsilon:
                    #     break

                    w = self.pidTheta.update(self.odometry.theta, goal_theta, time)
                    u = self.pidXY.update(current_displacement, goal_displacement, self.time.time())
                    
                    # Base Line
                    # The robot overshoots with pure base line strategy
                    self.create.drive_direct(int(100 + w), int(100 - w))

                    # Improved Solution 2
                    # self.create.drive_direct(int(u + w), int(u - w))


            # plotting for go-to-goal (goal_x, goal_x):
            plt.figure()
            f, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
            ax1.set_title("Angle")
            ax1.plot(result[:,0], result[:,1], label="odometry")
            ax1.plot(result[:,0], result[:,2], label="goal")
            ax1.grid()
            ax1.legend()

            ax2.set_title("Position")
            ax2.plot(result[:,3], result[:,4], label="odometry")
            ax2.scatter([goal_x], [goal_y], color="r", s=40, label="goal")
            ax2.axis("equal")
            ax2.grid()
            ax2.legend(loc="lower right")
            plt.savefig("lab6_position_solution2(0.5,0.5).png")