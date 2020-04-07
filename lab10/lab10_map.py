from PIL import Image, ImageDraw
import numpy as np
import math
import random

def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)

class Map:

    def __init__(self, file_name):
        self.img = Image.open(file_name)
        self.width, self.height = self.img.size
        self.draw = ImageDraw.Draw(self.img)
        self.thicker_border = 0

    def has_obstacle(self, x, y):
        return np.array(self.img.getpixel((clamp(x, 0, self.width-1), clamp(y, 0, self.height-1)))).sum() != 0

    def draw_line(self, pos1, pos2, color, width=2):
        self.draw.line((pos1[0], pos1[1], pos2[0], pos2[1]), fill=(color[0], color[1], color[2], 255), width=width)

    def save(self, file_name):
        self.img.save(file_name)

    def get_random_pos(self):
        while True:
            x = int(random.random() * self.width)
            y = int(random.random() * self.height)

            if not self.has_obstacle(clamp(x - self.thicker_border,0,self.width-1), y) and \
                not self.has_obstacle(x, clamp(y + self.thicker_border,0,self.height-1)) and \
                    not self.has_obstacle(clamp(x + self.thicker_border,0,self.width-1), \
                                          clamp(y + self.thicker_border,0,self.height-1)):
                return x, y

            # if not self.has_obstacle(x, y):
            #     return x, y

