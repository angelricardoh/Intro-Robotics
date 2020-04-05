# lab10.py
##############
# PSEUDOCODE (I hope...)
##############

# Set step_size
# Set initial point

# WHILE NOT (found_goal)
# Find random_point on map
# Identify closest point in TREE to random_point
# Draw an imaginary line from closest_point to random_point by finding angle b/w closest existing tree node and random loc
# Identify new_node along line
# IF  new_node is clear from obstacle:
# Add new_node(point_data, parent_node) to tree (which can just be an array of Nodes)
# Draw line into map

# IF new_node is close to goal:
# found_goal = true

# IF (found_goal):
# Backtrack from goal node to start node and recolor path


#################################################################################

from pyCreate2 import create2
import lab10_map
import math
import numpy as np

import odometry
import pid_controller


def get_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class Node(object):
    def __init__(self, loc, parent):
        self.loc = loc
        self.parent = parent


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab10_map.Map("lab10.png")
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

    def backtrack_to_start(self, node, color):
        path = []
        while node is not None:
            path.append(node.loc)
            print(node.loc)
            if node.parent is not None:
                print("test")
                self.map.draw_line(node.loc, node.parent.loc, color)
            node = node.parent

        return path

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
        # del waypoints[:10]
        print(waypoints)

        base_speed = 100
        start_time = self.time.time()

        # with open('output.csv', 'w') as f:
        for waypoint in waypoints:
            goal_x = waypoint[0]
            goal_y = 3.35 - waypoint[1]
            # goal_x = 3.25 - waypoint[1]
            # goal_y = waypoint[0] - 3.0
            print("Going to: (%s, %s)" % (goal_x, goal_y))

            while True:
                state = self.create.update()
                if state is not None:
                    # starts at (2.37, 0.33)
                    # self.odometry.x = 2.7
                    # self.odometry.y = 0.44
                    # self.odometry.theta = 1.57

                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    # f.write("{},{},{}\n".format(self.time.time() - start_time, theta, goal_theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # base version:
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))

                    # improved version 1: stop if close enough to goal
                    distance = math.sqrt(
                        math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.05:
                        break

                # improved version 2: fuse with velocity controller
                # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                # output_distance = self.pidDistance.update(0, distance, self.time.time())
                # if distance < 0.1:
                #     break
                # self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

    def run(self):

        # This is an example on how to check if a certain point in the given map is an obstacle
        # Each pixel corresponds to 1cm
        # print(self.map.has_obstacle(50, 60))

        # This is an example on how to draw a line
        # # (POINT 1, POINT 2, COLOR)
        # self.map.draw_line((0,0), (self.map.width, self.map.height), (255, 0, 0))
        # self.map.draw_line((0,self.map.height), (self.map.width, 0), (0, 255, 0))
        # self.map.save("lab10_rrt.png")

        # Comment these out if you don't want user-chosen input
        start_x = int(input("Enter a start x-coordinate or -1 for random: "))
        if start_x is not -1:
            start_y = int(input("Enter a start y-coordinate: "))

        goal_x = int(input("Enter a goal x-coordinate or -1 for random: "))
        if goal_x is not -1:
            goal_y = int(input("Enter a goal y-coordinate: "))

        STEP_SIZE = int(input("Step size? "))
        NUM_ITER = int(input("Number of iterations?"))
        # Default to random if no (x,y) specified
        start_point = Node(self.map.get_random_loc(), None)
        goal_point = Node(self.map.get_random_loc(), None)

        if start_x is not -1:
            start_point = Node((start_x, start_y), None)
        if goal_x is not -1:
            goal_point = Node((goal_x, goal_y), None)

        self.map.draw_line(goal_point.loc, (goal_point.loc[0] + 3, goal_point.loc[1] + 3), (0, 255, 0), width=10)

        print("Start at (%s, %s)" % (start_point.loc[0], start_point.loc[1]))

        found_goal = False

        nodes = list()
        nodes.append(start_point)
        # rand_loc_list = []
        count = 0
        while not found_goal and count < NUM_ITER:
            count += 1
            rand_loc = self.map.get_random_loc()
            # rand_loc_list.append(rand_loc)

            # Identify closest point in TREE to random_point
            min_dist = math.inf
            close_node = None
            # vec_node_dist = np.vectorize(lambda nodes_, rand_loc_: get_distance(x1=nodes_.loc[0], y1=nodes_.loc[1], x2=rand_loc_[0], y2=rand_loc_[1]))
            # nodes_dist = vec_node_dist(nodes, rand_loc_list)
            # min_dist = np.min(nodes_dist)
            # close_node = nodes[int(np.argmin(nodes_dist))]
            for n in nodes:
                temp_dist = get_distance(n.loc[0], n.loc[1], rand_loc[0], rand_loc[1])
                if temp_dist < min_dist:
                    min_dist = temp_dist
                    close_node = n

            # Draw an imaginary line from closest_point to random_point by finding angle b/w closest existing tree node and random loc
            theta = math.atan2(rand_loc[1] - close_node.loc[1], rand_loc[0] - close_node.loc[0])

            # Identify new_node along line
            new_x = int(close_node.loc[0] + STEP_SIZE * math.cos(theta))
            new_y = int(close_node.loc[1] + STEP_SIZE * math.sin(theta))

            # IF  new_node is clear from obstacle:
            # Add new_node(point_data, parent_node) to tree (which can just be an array of Nodes)
            # Draw new line segment into map
            if not self.map.has_obstacle(new_x, new_y):
                new_node = Node((new_x, new_y), close_node)

                # DEBUG
                if close_node is None:
                    bad_error = True

                print(new_node.loc)

                nodes.append(new_node)
                self.map.draw_line(new_node.parent.loc, new_node.loc, (255, 0, 0))

                # NOTE: this should only execute if we actually find a new point
                # IF new_node is close to goal:
                # found_goal = true
                if get_distance(x1=new_node.loc[0], x2=goal_point.loc[0], y1=new_node.loc[1],
                                y2=goal_point.loc[1]) < 15:
                    goal_point.parent = new_node
                    found_goal = True

        # Backtrack from goal node to start node and recolor path
        if found_goal:
            print("Goal loc: (%s,%s)" % goal_point.loc)
            self.backtrack_to_start(goal_point, (0, 255, 0))

            path = self.backtrack_to_start(goal_point, (0, 255, 0))

            self.map.save("lab10_rrt.png")

            path.reverse()
            print(path)
            self.follow_path(path)

        else:
            print("Error -- exhausted nodes before completion...")
