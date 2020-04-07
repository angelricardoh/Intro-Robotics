from pyCreate2 import create2
import lab10_map
import math
import numpy as np

import odometry
import pid_controller


class Node:
    parent = None
    cost = 0

    def __init__(self, pos, inclination=None, explored=None, edges=None):
        self.pos = pos
        self.inclination = inclination
        self.explored = explored
        self.edges = edges

    def description(self):
        return "Node pos: " + self.pos + " inclination: " + str(self.inclination) + \
               " explored: " + str(self.explored) + " edges: " + str(self.edges)


start_point = Node((270, 308))
goal_point = Node((40, 120))
STEP_SIZE = 30
NUM_ITER = 500000

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab10_map.Map("lab10.png")
        self.map.thicker_border = 10
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)

    def run(self):
        print("Start at (%s, %s)" % (start_point.pos[0], start_point.pos[1]))
        goal_localized = False

        nodes = [start_point]
        count = 0

        while not goal_localized and count < NUM_ITER:
            rand_pos = self.map.get_random_pos()

            min_dist = math.inf
            nearest_node = None

            for n in nodes:
                temp_dist = get_distance(n.pos[0], n.pos[1], rand_pos[0], rand_pos[1])
                if temp_dist < min_dist:
                    min_dist = temp_dist
                    nearest_node = n

            theta = math.atan2(rand_pos[1] - nearest_node.pos[1], rand_pos[0] - nearest_node.pos[0])

            new_x = int(nearest_node.pos[0] + STEP_SIZE * math.cos(theta))
            new_y = int(nearest_node.pos[1] + STEP_SIZE * math.sin(theta))

            rand_pos_x = rand_pos[0]
            rand_pos_y = rand_pos[1]
            diff_x = 0
            diff_y = 0

            # make thicker obstacles from left and top
            if new_y > rand_pos_y:
                diff_y = self.map.thicker_border

            if new_y < rand_pos_y:
                diff_y = self.map.thicker_border

            count += 1

            if not self.map.has_obstacle(new_x, new_y) and not self.map.has_obstacle(new_x - diff_x, new_y) and \
                    not self.map.has_obstacle(new_x, new_y + diff_y) and \
                    not self.map.has_obstacle(new_x - diff_x, new_y + diff_y):
                new_node = Node((new_x, new_y))
                new_node.parent = nearest_node
                new_node.inclination = theta

                # print(new_node)

                nodes.append(new_node)
                self.map.draw_line(new_node.parent.pos, new_node.pos, (0, 0, 0))

                if get_distance(x1=new_node.pos[0], x2=goal_point.pos[0], y1=new_node.pos[1],
                                y2=goal_point.pos[1]) < 15:
                    goal_point.parent = new_node
                    goal_localized = True

        if goal_localized:
            print("Goal pos: (%s,%s)" % goal_point.pos)

            self.map.save("lab10_rrt.png")
            solution = goal_point
            solution_path = [solution.pos]
            while solution.parent:
                if solution.parent is not None:
                    self.map.draw_line(solution.pos, solution.parent.pos, (0, 255, 0))
                solution = solution.parent
                solution_path.insert(0, solution.pos)

            self.map.save("lab10_shortest_path.png")

            self.follow_path(solution_path)
        else:
            print("No solution found")

    def follow_path(self, path):
        self.create.start()
        self.create.safe()

        self.odometry.x = 2.7
        self.odometry.y = 0.33
        self.odometry.theta = math.pi / 2

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        waypoints = [[n[0] / 100, n[1] / 100] for n in path]
        
        print("waypoints")
        del waypoints[:1]
        print(waypoints)

        base_speed = 100
        start_time = self.time.time()

        for waypoint in waypoints:
            goal_x = waypoint[0]
            goal_y = 3.25 - waypoint[1]

            print("Going to: (%s, %s)" % (goal_x, goal_y))

            while True:
                state = self.create.update()
                if state is not None:

                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))

                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # base version:
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))

                    # improved version 1: stop if close enough to goal
                    distance = math.sqrt(
                        math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.1:
                        break


def get_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)