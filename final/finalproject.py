from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab10_map
import lab8_map
import particle_filter
import rrt
import random
from particle_filter import ParticleFilter
import numpy as np
from my_arm_robot import MyArmRobot, Joints


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self._map = lab8_map.Map("lab8_map.json")
        # self.map = lab10_map.Map("configuration_space.png")
        self.map = lab10_map.Map("config2.png")
        self.rrt = rrt.RRT(self.map)
        self.min_dist_to_localize = 0.1
        self.min_theta_to_localize = math.pi / 4
        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        # self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # self.pidDistance = pid_controller.PIDController(300, 0, 20, [0, 0], [-200, 200], is_angle=False)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self._map, 5000, 0.06, 0.15, 0.2)

        _ = self.create.sim_get_position()
        self.my_arm_robot = MyArmRobot(factory.create_kuka_lbr4p(), self.time)

        self.joint_angles = np.zeros(7)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break
   
    def positioning(self):
        # for _ in range(21):
        angle = math.pi/5
        counter = 0
        while True:
            
            distance = self.sonar.get_distance()

            self.go_to_angle(self.odometry.theta + angle)
            print('theta:', self.odometry.theta, math.degrees(self.odometry.theta))
            self.pf.measure(distance, 0)
            self.visualize()
            counter += 1
            self.time.sleep(0.01)

            if self.isLocalized():
                self.create.drive_direct(0,0)
                return True

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def isLocalized(self):
        x, y, theta = self.pf.get_estimate()
        theta %= (2 * math.pi)
        self.odometry.theta = self.odometry.theta % (2 * math.pi)
        # ground_truth = self.create.sim_get_position()
        # self.odometry.x = ground_truth[0]
        # self.odometry.y = ground_truth[1]

        dist_position_to_goal = math.sqrt(
            (self.odometry.x - x) ** 2 + (self.odometry.y - y) ** 2)
        print("self.odometry.x " + str(self.odometry.x) + " self.odometry.y " + str(self.odometry.y) +
              " self.odometry.theta " + str(self.odometry.theta))
        print("self.pf.x " + str(x) + " self.pf.y " + str(y) +
              " self.pf.theta " + str(theta))

        diff_theta_to_goal = abs(self.odometry.theta - theta)
        print("EUCLIDEAN_DISTANCE ", dist_position_to_goal)
        print("DISTANCE_theta ", diff_theta_to_goal)

        isLocalized = dist_position_to_goal < self.min_dist_to_localize and diff_theta_to_goal < self.min_theta_to_localize
        if isLocalized:
            self.start_x = x 
            self.start_y = y 
            print("start_x")
            print (x*100)
            print("start_y")
            print (y*100)
            print("Localized")
        return isLocalized

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        self.arm.open_gripper()

        self.time.sleep(4)

        self.arm.close_gripper()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.visualize()
        self.virtual_create.enable_buttons()
        self.visualize()

        origin = (0.5, 0.5, 0.1, 0)             # partical origin
        noise = (0.01, 0.05, 0.1)     # sd for (distance, theta, sonar)
        
        self.odometry.x = origin[0]
        self.odometry.y = origin[1]

        self.min_dist_to_localize = 0.45
        self.min_theta_to_localize = math.pi / 2
    

        # print(self.min_theta_to_localize)
        # while True:
        #     i = 1
        # self.pf = ParticleFilter(origin, noise, 1000, self.map)
        # self.virtual_create.set_point_cloud(self.pf.get_particle_list())

        isLocal = False

        while not isLocal:
            isLocal = self.positioning()
            if isLocal ==True:
                break
        
        self.go_to_angle(0)
        _ = self.create.sim_get_position

        # self.rrt.build((self.start_x*100, 300-(self.start_y*100)), 4000, 10)

        # ONLY MODIFY THESE VALUES.  THIS IS END LOCATION OF THE ROBOT (OR WHERE IT SHOULD BE)
        arm_x = 1.55
        arm_y = 2.55

        # print("start")
        # print(self.start_x)
        # print(self.start_y)
        # print("ground_truth")
        # ground_truth = self.create.sim_get_position()
        # print(ground_truth[0])
        # print(ground_truth[1])
        # self.start_x = ground_truth[0]
        # self.start_y = ground_truth[1]
        # self.odometry.x = self.start_x
        # self.odometry.y = self.start_y

        self.rrt.build((arm_x*100, 300-(arm_y*100)), 4000, 10)

        x_goal = self.rrt.nearest_neighbor((self.start_x*100, 300-(self.start_y*100)))
        path = self.rrt.shortest_path(x_goal)

        # self.rrt.build((self.start_x*100, 300-(self.start_y*100)), 4000, 10)

        # x_goal = self.rrt.nearest_neighbor((167,300-230))
        # path = self.rrt.shortest_path(x_goal)
        # self.rrt.build((100, 270), 4000, 10)

        # x_goal = self.rrt.nearest_neighbor((100, 50))
        # path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        self.map.save("PATH_TO_FOLLOW.png")

        # execute the path (essentially waypoint following from lab 6)
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.odometry.x = self.start_x
        self.odometry.y = self.start_y
        self.odometry.theta = 0
        base_speed = 100

        path.pop()
        path.pop()
        path.pop()
        path.pop()
        path.reverse()
        for p in path:
            goal_x =p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0
            print(goal_x, goal_y)
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

                    # Just PIDTheta
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

                    _ = self.create.sim_get_position()

                    # goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    # theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    # self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                    # # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                    #
                    # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    #
                    # # if goal_x <= 1:
                    # output_distance = self.pidDistance.update(0, distance, self.time.time())
                    # self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

                    if distance < 0.05:
                        # self.odometry.x = -0.05 + p.state[0] / 100.0
                        # self.odometry.y = 3 - p.state[1] / 100.0
                        break



        self.create.drive_direct(0,0)
        self.go_to_angle(0)

        print("odometry")
        print(self.odometry.x)
        print(self.odometry.y)
        ground_truth = self.create.sim_get_position()
        print("ground_truth")
        print(ground_truth[0])
        print(ground_truth[1])

        print("particle_filter")
        x, y, theta = self.pf.get_estimate()
        print(x)
        print(y)
        val_1 = '%.2f' % (ground_truth[0])
        val_2 = '%.2f' % (ground_truth[1])
        print("differences")
        print(1.6 - float(val_1))
        print(3.4 - float(val_2))
        self.my_arm_robot.set_create_coordinates(1.6 - float(val_1), 3.4 - float(val_2))
        self.my_arm_robot.grab_cup_and_go_to_initial_position()
        self.my_arm_robot.place_cup_in_shelf_one()
