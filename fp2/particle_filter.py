# import numpy as np
# import math
# import scipy.stats
# from numpy.random import choice
# import copy


# class Particle:
#     def __init__(self, x, y, theta, ln_p):
#         self.x = x
#         self.y = y
#         self.theta = theta
#         self.ln_p = ln_p


# class ParticleFilter:
#     def __init__(self, map, num_particles, translation_variance, rotation_variance, measurement_variance):
#         self._particles = []
#         self._map = map
#         self._translation_variance = translation_variance
#         self._rotation_variance = rotation_variance
#         self._measurement_variance = measurement_variance
#         # sample possible states for particles uniformly
#         p = 1.0 / num_particles
#         for i in range(0, num_particles):
#             self._particles.append(Particle(
#                 np.random.uniform(map.bottom_left[0], map.top_right[0]),
#                 np.random.uniform(map.bottom_left[1], map.top_right[1]),
#                 # choice([-math.pi, -math.pi/2, 0, math.pi/2]),
#                 np.random.uniform(0, 2*math.pi),
#                 # 0,
#                 math.log(p)))

#     def move_by(self, x, y, theta):
#         for particle in self._particles:
#             particle.x += x + np.random.normal(0.0, self._translation_variance)
#             particle.y += y + np.random.normal(0.0, self._translation_variance)
#             particle.theta += theta + np.random.normal(0.0, self._rotation_variance)
#             # bound check
#             particle.x = min(self._map.top_right[0], max(particle.x, self._map.bottom_left[0]))
#             particle.y = min(self._map.top_right[1], max(particle.y, self._map.bottom_left[1]))
#             particle.theta = math.fmod(particle.theta, 2 * math.pi)
#         # self.particles_for_map = self._particles

#     def measure(self, z, servo_angle_in_rad):
#         for particle in self._particles:
#             # compute what the distance should be, if particle position is accurate
#             distance = self._map.closest_distance([particle.x, particle.y], particle.theta + servo_angle_in_rad)
#             print(particle.theta + servo_angle_in_rad)
#             print([particle.x, particle.y])
#             print(distance)
#             # compute the probability P[measured z | robot @ x]
#             p = scipy.stats.norm(distance, self._measurement_variance).pdf(z)
#             if p == 0:
#                 p_measured_z_if_robot_at_x = float("-inf")
#             else:
#                 p_measured_z_if_robot_at_x = math.log(p)
#             # compute the probability P[robot@x | measured]
#             #    NOTE: This is not a probability, since we don't know P[measured z]
#             #          Hence we normalize afterwards
#             particle.ln_p = p_measured_z_if_robot_at_x + particle.ln_p
#         # normalize probabilities (take P[measured z into account])
#         probabilities = np.array([particle.ln_p for particle in self._particles])
#         a = scipy.misc.logsumexp(probabilities)
#         probabilities -= a
#         for j in range(0, len(probabilities)):
#             self._particles[j].ln_p = probabilities[j]

#         # resample
#         self._particles = choice(self._particles, len(self._particles), p=[math.exp(particle.ln_p) for particle in self._particles])
#         self._particles = [copy.copy(particle) for particle in self._particles]

#     def get_estimate(self):
#         weights = [math.exp(particle.ln_p) for particle in self._particles]
#         x = np.average([particle.x for particle in self._particles], weights=weights)
#         y = np.average([particle.y for particle in self._particles], weights=weights)
#         theta = np.average([particle.theta for particle in self._particles], weights=weights)
#         return x, y, theta


import numpy as np
import math
import scipy.stats
import scipy
import scipy.misc
from scipy.special import logsumexp
from numpy.random import choice
import copy
from scipy import stats


class Particle:
    def __init__(self, x, y, theta, ln_p):
        self.x = x
        self.y = y
        self.theta = theta
        self.ln_p = ln_p


class ParticleFilter:
    def __init__(self, map, num_particles, translation_variance, rotation_variance, measurement_variance):
        self._particles = []
        self._map = map
        
        self.bound = self._map.bottom_left, self._map.top_right
        self._translation_variance = translation_variance
        self._rotation_variance = rotation_variance
        self._measurement_variance = measurement_variance
        self.num_particles = num_particles
        # sample possible states for particles uniformly
        p = 1.0 / num_particles
        for i in range(0, num_particles):
            self._particles.append(Particle(
                np.random.uniform(map.bottom_left[0], map.top_right[0]),
                np.random.uniform(map.bottom_left[1], map.top_right[1]),
                # choice([-math.pi, -math.pi/2, 0, math.pi/2]),
                np.random.uniform(0, 2*math.pi),
                # 0,
                math.log(p)))

    def get_particle_list(self):
        p_list = list()
        for p in self._particles:
            p_list += [p.x, p.y, 0, p.theta]
        return p_list
    def move_by(self, x, y, theta):
        for particle in self._particles:
            particle.x += x + np.random.normal(0.0, self._translation_variance)
            particle.y += y + np.random.normal(0.0, self._translation_variance)
            particle.theta += theta + np.random.normal(0.0, self._rotation_variance)
            # bound check
            particle.x = min(self._map.top_right[0], max(particle.x, self._map.bottom_left[0]))
            particle.y = min(self._map.top_right[1], max(particle.y, self._map.bottom_left[1]))
            particle.theta = math.fmod(particle.theta, 2 * math.pi)
        # self.particles_for_map = self._particles

    def measure(self, z, servo_angle_in_rad):
        for particle in self._particles:
            # compute what the distance should be, if particle position is accurate
            distance = self._map.closest_distance([particle.x, particle.y], particle.theta + servo_angle_in_rad)
            # print(particle.theta + servo_angle_in_rad)
            # print([particle.x, particle.y])
            # print(distance)
            # compute the probability P[measured z | robot @ x]
            p = scipy.stats.norm(distance, self._measurement_variance).pdf(z)
            if p == 0:
                p_measured_z_if_robot_at_x = float("-inf")
            else:
                p_measured_z_if_robot_at_x = math.log(p)
            # compute the probability P[robot@x | measured]
            #    NOTE: This is not a probability, since we don't know P[measured z]
            #          Hence we normalize afterwards
            particle.ln_p = p_measured_z_if_robot_at_x + particle.ln_p
        # normalize probabilities (take P[measured z into account])
        probabilities = np.array([particle.ln_p for particle in self._particles])
        a = scipy.special.logsumexp(probabilities)
        probabilities -= a
        for j in range(0, len(probabilities)):
            self._particles[j].ln_p = probabilities[j]

        # resample
        self._particles = choice(self._particles, len(self._particles), p=[math.exp(particle.ln_p) for particle in self._particles])
        self._particles = [copy.copy(particle) for particle in self._particles]

    def get_estimate(self):
        weights = [math.exp(particle.ln_p) for particle in self._particles]
        x = np.average([particle.x for particle in self._particles], weights=weights)
        y = np.average([particle.y for particle in self._particles], weights=weights)
        theta = np.average([particle.theta for particle in self._particles], weights=weights)
        return x, y, theta

    def movement(self, d, theta):
        for p in self.particles:
            d_prime = d + np.random.normal(0, self._translation_variance)
            theta_prime = theta + np.random.normal(0, self._rotation_variance)

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
        if any(two_line_intersect(A, B, (w[0][0], w[0][1]), (w[1][0], w[1][1])) for w in self._map.get_walls()):
            return True

    def sensing(self, z):
        for p in self.particles:
            distance = self._map.closest_distance((p.x, p.y), p.theta)
            p.weight = stats.norm.pdf(z, distance, self._measurement_variance)
            # p.weight = stats.norm.pdf(z, distance, self.noise[2]) + p.weight

        self.resample()
        self.estimation()


    def resample(self):
        self.normalize()
        weights = [p.weight for p in self._particles]
        total_weight = sum(weights)
        choice = np.random.choice(self._particles, size=self.num_particles, replace=True, p=weights/total_weight)
        copy = []
        for p in choice:
            copy.append(Particle(x=p.x,y=p.y,theta=p.theta,ln_p=p.weight))
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
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
 
    