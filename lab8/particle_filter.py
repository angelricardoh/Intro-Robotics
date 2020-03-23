import math
import numpy
import enum
import numpy as np
from scipy.stats import norm
from scipy.special import logsumexp
import copy


class Command(enum.Enum):
    straight = 1
    turn_left = 2
    turn_right = 3


class Particle:
    def __init__(self, pos_x, pos_y, theta, map, weight, distance, sigma_sensor, sigma_direction, sigma_distance, id, initial_prob):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.theta = theta
        self.map = map
        self.weight = weight
        self.distance = distance
        # self.sigma_sensor = np.sqrt(sigma_sensor)
        # self.sigma_direction = np.sqrt(sigma_direction)
        # self.sigma_distance = np.sqrt(sigma_distance)
        self.sigma_sensor = sigma_sensor
        self.sigma_direction = sigma_direction
        self.sigma_distance = sigma_distance
        self.id = id
        self.prev_posterior_log_prob = initial_prob

    def __repr__(self):
        # return "Particle id:% d, x:% f, y:% f, θ:% f, weight:% f, prev_prob:% f" \
        #        % (self.id, self.pos_x, self.pos_y, self.theta, self.weight, self.prev_posterior_log_prob)
        return "Particle id:% d, x:% f, y:% f, θ:% f, weight:% f, current_prob:% f" \
                   % (self.id, self.pos_x, self.pos_y, self.theta, self.weight, self.calculate_partial_posterior_prob())

    def update_sensor_reading(self, sensor_measurement_z):
        self.sensor_reading = sensor_measurement_z

    def calculate_partial_posterior_prob(self):
        print("[particle id: {}, x: {}, y: {}, theta: {}]".format(self.id, self.pos_x, self.pos_y, math.degrees(self.theta)))
        location = self.map.closest_distance((self.pos_x, self.pos_y), self.theta)
        if location is None or location == 0:
            return self.prev_posterior_log_prob

        prob_sensor_given_robot_loc = norm.pdf(self.sensor_reading, loc=location, scale=self.sigma_sensor)

        posterior_probability = math.log(prob_sensor_given_robot_loc) + self.prev_posterior_log_prob

        return posterior_probability

    def move(self, desired_theta=None):
        # model (maybe variance σ(square) goes here instead of standard deviation)
        print("theta: " + str(desired_theta))
        if desired_theta is None:
            desired_theta = self.theta
        normal_random_variable_sample_direction = np.random.normal(0, self.sigma_direction, 1)[0]
        print("[particle id: {}, nrvsdirection: {}]".format(self.id, normal_random_variable_sample_direction))
        theta_prime = desired_theta + normal_random_variable_sample_direction
        # theta_prime %= 2 * np.pi

        normal_random_variable_sample_distance = np.random.normal(0, self.sigma_distance, 1)[0]
        print("[particle id: {}, nrvsdistance: {}]".format(self.id, normal_random_variable_sample_distance))
        distance_prime = self.distance + normal_random_variable_sample_distance

        # propagate
        self.pos_x = clamp(self.pos_x + distance_prime * np.cos(theta_prime), self.map.bottom_left[0], self.map.top_right[0])
        self.pos_y = clamp(self.pos_y + distance_prime * np.sin(theta_prime), self.map.bottom_left[1], self.map.top_right[1])
        self.theta = theta_prime


class ParticleFilter:
    def __init__(
            self,
            map,
            virtual_create,
            num_particles, distance, sigma_sensor, sigma_theta, sigma_distance):
        self.map = map
        self.virtual_create = virtual_create
        self.distance = distance
        self.sigma_sensor = sigma_sensor,
        self.sigma_direction = sigma_theta,
        self.sigma_distance = sigma_distance

        self.particles = []
        self.weights = [1/num_particles] * num_particles

        for index in range(0, num_particles):
            pos_x = np.random.uniform(0, 3)
            pos_y = np.random.uniform(0, 3)
            theta = np.random.uniform(0, 2 * np.pi)
            # print("[x: {}, y: {}, theta: {}]".format(pos_x, pos_y, math.degrees(theta)))

            self.particles.append(
                Particle(
                    pos_x=pos_x,
                    pos_y=pos_y,
                    theta=theta,
                    map=self.map,
                    weight=(1 / num_particles),
                    distance=self.distance,
                    sigma_sensor=self.sigma_sensor,
                    sigma_direction=self.sigma_direction,
                    sigma_distance=self.sigma_distance,
                    id=index,
                    initial_prob=math.log(1 / num_particles)
                )
            )

        self.current_movement = None
        self.current_distance = None

    def movement(self, current_command: Command, desired_angle=None):
        for particle in self.particles:
            particle.move(desired_angle)
        self.estimation()
        self.draw_particles()


    def sensing(self, sensor_measurement_z: float):
        for particle_index in range(0, len(self.particles)):
            particle = self.particles[particle_index]
            particle.update_sensor_reading(sensor_measurement_z)
            self.weights[particle_index] = particle.calculate_partial_posterior_prob()
            # print(particle)

        # print("weights before adding logsumexp: ", self.weights)

        # Compute N and add to have total posterior probability
        self.weights += logsumexp(self.weights)
        # self.weights -= logsumexp(self.weights)
        # n_effective = logsumexp(self.weights + self.weights)
        # print("Neff = %.4f" % np.exp(n_effective))

        print("weights before adding logsumexp")
        self.print_particles()

        np.vectorize(set_prev_prob)(self.particles, self.weights)

        print("particles after vectorize")
        self.print_particles()

        # if n_effective < self.n_threshold:
        #     self.resample_particles()
        # self.resample_particles()
        self.estimation()
        self.draw_particles()

    # Analyze
    def estimation(self):
        x_avg = np.average(np.vectorize(lambda particle: particle.pos_x)(self.particles))

        y_avg = np.average(np.vectorize(lambda particle: particle.pos_y)(self.particles))
        theta_avg = np.average(np.vectorize(lambda particle: particle.theta)(self.particles))

        # draw the estimated position and all other particles
        self.virtual_create.set_pose((x_avg, y_avg, 0.1), theta_avg)


    # Analyze
    def resample_particles(self):
        index = np.random.choice(np.arange(0, len(self.particles)), len(self.particles), replace=True,
                                 p=np.exp(self.weights))
        particles = []
        for i in index:
            particles.append(copy.deepcopy(self.particles[i]))

        return particles

    def draw_particles(self):
        data = []

        for particle in self.particles:
            data.extend([particle.pos_x, particle.pos_y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def print_particles(self):
        for particle in self.particles:
            print(particle)

def clamp(value, range_min, range_max):
    return value
    # return max(min(value, range_max), range_min)


def set_prev_prob(particle, weight):
    particle.prev_posterior_log_prob = weight