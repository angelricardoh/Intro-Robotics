import math
import numpy
import enum
import numpy as np


class Command(enum.Enum):
    straight = 1
    turn_left = 2
    turn_right = 3


class Particle:
    def __init__(self, pos_x, pos_y, angle):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.angle = angle


class ParticleFilter:
    def __init__(
            self,
            map,
            virtual_create,
            num_particles):
        self.map = map
        self.virtual_create = virtual_create

        self.particles = []
        # for num in num_particles:
        positions_x = np.random.uniform(0, 3, num_particles)
        positions_y = np.random.uniform(0, 3, num_particles)
        raws = np.random.uniform(0, 2 * np.pi, num_particles)
        for index in range(0, num_particles):
            self.particles.append(Particle(pos_x=positions_x[index], pos_y=positions_y[index], angle=raws[index]))

        self.current_movement = None
        self.current_distance = None

    def movement(self, current_movement: Command):
        # TODO: Particle filter algorithm steps
        self.current_movement = current_movement

    def sensing(self, current_distance: float):
        # TODO: Particle filter at the end of lecture 16
        self.current_distance = current_distance

    def draw_particles(self):
        data = []

        # get position data from all particles
        for particle in self.particles:
            data.extend([particle.pos_x, particle.pos_y, 0.1, particle.angle])

            # paint all particles in simulation
            self.virtual_create.set_point_cloud(data)
