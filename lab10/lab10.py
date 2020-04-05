from pyCreate2 import create2
import lab11_map


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab11_map.Map("lab11.png")

    def run(self):
        # This is an example on how to check if a certain point in the given map is an obstacle
        # Each pixel corresponds to 1cm
        print(self.map.has_obstacle(50, 60))

        # This is an example on how to draw a line
        self.map.draw_line((0,0), (self.map.width, self.map.height), (255, 0, 0))
        self.map.draw_line((0,self.map.height), (self.map.width, 0), (0, 255, 0))
        self.map.save("lab11_rrt.png")
