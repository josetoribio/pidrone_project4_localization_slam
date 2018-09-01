import numpy as np
import math
import copy

NUM_PARTICLES = 75
PROB_THRESHOLD = 0.005
UPDATE_VARIANCE = 0.1
TARGET = [-5, 5]


class Particle:
    def __init__(self, x, y):
        self.pose = [x, y]
        self.weight = PROB_THRESHOLD


class ParticleSet:
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = [Particle(0.0, 0.0) for _ in range(NUM_PARTICLES)]

    def update(self):
        for i in range(self.num_particles):
            np.random.seed(i)
            noise_x = np.random.normal(0, UPDATE_VARIANCE)
            noise_y = np.random.normal(0, UPDATE_VARIANCE)
            self.particles[i].pose[0] += noise_x
            self.particles[i].pose[1] += noise_y

    def weight(self):
        for p in self.particles:
            dist = distance(p.pose[0], p.pose[1], TARGET[0], TARGET[1])
            p.weight = 1. / dist

    def resample(self):
        weights = np.array([p.weight for p in self.particles])
        weights_sum = np.sum(weights)
        new_particles = []

        normal_weights = weights / float(weights_sum)
        samples = np.random.multinomial(self.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                p = copy.deepcopy(self.particles[i])
                p.weight = PROB_THRESHOLD
                new_particles.append(p)

        self.particles = new_particles


def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


particles = ParticleSet(NUM_PARTICLES)

pose_path = '/users/luke/Desktop/SLAM/sample_poses.txt'
weights_path = '/users/luke/Desktop/SLAM/sample_weights.txt'

try:
    pp = open(pose_path, 'w')
    wp = open(weights_path, 'w')
    for _ in range(500):
        particles.update()
        particles.weight()

        wp.write(str([p.weight for p in particles.particles]) + '\n')

        particles.resample()

        for p in particles.particles:
            pp.write(str(p.pose[0]) + '\n' + str(p.pose[1]) + '\n')
finally:
    pp.close()
    wp.close()


