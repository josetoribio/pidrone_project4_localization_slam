import numpy as np
import math
import copy

NUM_PARTICLES = <choose a number>
UPDATE_VARIANCE = 0.1
TARGET = <choose a location with both x,y within 0-10>


class Particle:
    """
    Class to hold a particle in your filter. Should store its position (x,y)
    and its weight. Once optimized in problem 6, it should store the set of
    all weights and poses as numpy arrays. Depending on how you implement the
    optimization, it may also need fields or methods for retrieving its own
    weight and pose.
    """
    def __init__(self):


class ParticleSet:
    """
    Class to hold the set of particles and the methods which will operate on them.
    You may want to also store the target pose here as well. Remember, the goal
    is for the (x,y) positions of the set of particles to converge on the
    target pose.You must implement a motion model, measurement_model,
    resampling method, and a method for upating the filter. Detailed instructions
    are in the "Localization Assignment" page in the online textbook.
    """
    def __init__(self, num_particles):
        """
        Takes the number of particles as input and creates a set of particles at
        random positions.
        """

    def motion_model(self):
        """
        Adds some random Gaussian noise to the x and y positions of each
        particle in the filter. Be sure that the noise is different for each
        particle. Use UPDATE_VARIANCE for the variance when you sample from the
        Gaussian
        Hint: try numpy.random.normal
        """

    def measurement_model(self):
        """
        Sets the weight of each particle inversely proportional to the
        particle’s distance from the desired pose. This weighting scheme will
        ensure that particles closest to the target have higher weights, and
        thus are resampled with a higher probability.
        """

    def resample_particles(self):
        """
        Uses the particle weights to resample the particles and create a new set.
        The goal is for each particle to be resampled with a probability that
        corresponds to its weight, therefore the particle set converges on the
        desired pose (particles closest to the target pose are more likely to be
        resampled). You will want to use copy.deepcopy to create a copy of each
        re-sampled particle.

        Hint: try using numpy.random.multinomial to generate the set of new
        samples.
        """

    def update_filter(self):
        """
        Updates the filter by calling motion_model, measurement_model, then
        resample_particles.
        """





###############################################################################
# Do not edit below this line
###############################################################################

particles = ParticleSet(NUM_PARTICLES)

pose_path = 'particle_filter_data.txt'

try:
    pp = open(pose_path, 'w')
    for _ in range(500):
        particles.update()
        particles.weight()

        particles.resample()

        for p in particles.particles:
            pp.write(str(p.pose[0]) + '\n' + str(p.pose[1]) + '\n')
finally:
    pp.close()

