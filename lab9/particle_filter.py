import numpy as np
from scipy import stats
import lab8_map
import math


class Particle:
    def __init__(self, x, y, z, theta, weight):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.weight = weight

class ParticleFilter:
    def __init__(self, position, noise, num_of_particles, my_map):
        self.xyz = position[:-1]        # position (x, y, z, theta)
        self.theta = position[-1]
        self.noise = noise              # sd for (distance, theta, sonar)
        self.particles = []
        self.num_of_particles = num_of_particles
        self.map = my_map
        self.bound = my_map.bottom_left, my_map.top_right

        for _ in range(num_of_particles):
            x = np.random.uniform(self.bound[0][0], self.bound[1][0])
            y = np.random.uniform(self.bound[0][1], self.bound[1][1])
            z = self.xyz[2]
            theta = np.random.uniform(0, 2 * math.pi)
            initial_weight = 1 / num_of_particles
            self.particles.append(Particle(x, y, z, theta, initial_weight))

        self.estimation()

    def movement(self, d, theta):
        for p in self.particles:
            d_prime = d + np.random.normal(0, self.noise[0])
            theta_prime = theta + np.random.normal(0, self.noise[1])

            prev_pos = (p.x, p.y)

            p.x += d_prime*math.cos(p.theta + theta_prime)
            p.y += d_prime*math.sin(p.theta + theta_prime)
            p.theta = (p.theta + theta) % (2 * math.pi)

            if self.out_side_map(p) or self.ghosting_wall(prev_pos, (p.x, p.y)):
                self.particles.remove(p)

        self.estimation()


    def out_side_map(self, p):
        if p.x > self.bound[1][0] or p.x < self.bound[0][0] or \
           p.y > self.bound[1][1] or p.y < self.bound[0][1]:
            return True

    def ghosting_wall(self, old_pos, new_pos):
        def two_line_intersect(A, B, C, D):     # if line AB intersect with line CD
            def ccw(A,B,C):                     # did not check colinearity
                return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])      
            return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        A = old_pos
        B = new_pos
        if any(two_line_intersect(A, B, (w[0][0], w[0][1]), (w[1][0], w[1][1])) for w in self.map.get_walls()):
            return True

    def sensing(self, z):
        for p in self.particles:
            distance = self.map.closest_distance((p.x, p.y), p.theta)
            p.weight = stats.norm.pdf(z, distance, self.noise[2]) #+ p.weight
        self.resample()
        self.estimation()


    def resample(self):
        self.normalize()
        weights = [p.weight for p in self.particles]
        total_weight = sum(weights)
        choice = np.random.choice(self.particles, size=self.num_of_particles, replace=True, p=weights/total_weight)
        copy = []
        for p in choice:
            copy.append(Particle(x=p.x,y=p.y,z=p.z,theta=p.theta,weight=p.weight))
        self.particles = copy


    def normalize(self):
        n = sum(p.weight for p in self.particles)
        for p in self.particles:
            p.weight /= n

    def estimation(self):
        x = np.average([p.x for p in self.particles])
        y = np.average([p.y for p in self.particles])
        z = np.average([p.z for p in self.particles])
        theta = np.average([p.theta for p in self.particles])
        self.xyz = (x, y, z)
        self.theta = theta

    def get_particle_list(self):
        p_list = list()
        for p in self.particles:
            p_list += [p.x, p.y, p.z, p.theta]
        return p_list