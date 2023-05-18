"""
slam_helper.py
test
Implements fastSLAM for the pidrone
"""

import numpy as np
import math
import utils
import copy
import cv2

# set this to true to write your SLAM output to a file to be animated
WRITE_TO_FILE  = True

# ----- camera parameters DO NOT EDIT ------- #
CAMERA_SCALE = 290.
CAMERA_WIDTH = 320.
CAMERA_HEIGHT = 240.
MATCH_RATIO = 0.7

# ----- SLAM parameters ------- #
PROB_THRESHOLD = 0.005
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT
KEYFRAME_YAW_THRESHOLD = 0.175

# ----- edit to where you want the SLAM data written --------- #
path = 'pose_data.txt'


class Particle:
    """
    attributes:
    pose:           a list giving the robot's position (x, y, z, yaw)
    landmarks:      a list of landmark objects
    weight:         the current weight for the particle
    """

    def __init__(self, x, y, z, yaw):
        self.pose = [x, y, z, yaw]
        self.landmarks = []
        self.weight = PROB_THRESHOLD

    def __str__(self):
        return "Pose: " + str(self.pose) + " Weight: " + str(self.weight)


class FastSLAM:
    def __init__(self):
        self.particles = None
        self.num_particles = None
        self.weight = PROB_THRESHOLD

        self.z = 0
        self.perceptual_range = 0.0

        # key points and descriptors from the most recent keyframe
        self.key_kp = None
        self.key_des = None

        if WRITE_TO_FILE:
            self.file = open(path, 'w')

        # --------------- openCV parameters --------------------- #
        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        # ------- parameters for noise on observations ----------- #
        self.sigma_d = 3
        self.sigma_p = .30
        self.sigma_observation = np.array([[self.sigma_d ** 2, 0], [0, self.sigma_p ** 2]])

        # ------- parameters for noise on motion updates --------- #
        sigma_vx = 2
        sigma_vy = 2
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def generate_particles(self, num_particles):
        """
        Creates the initial set of particles for SLAM
        Each particle should be initialized at (0,0) since we build the map relative to the drone's
        initial position, but with some noise.

        :param num_particles: the number of particles to generate
        """
        
        ##############################################################################
        # TODO Initialize the set of particles for FASTSLAM with x,y poses around 0,
        #      z initialized to self.z, and yaw intialized to pi. Be sure to add noise
        #      to the z,y, and yaw initial estimates!
        # Be sure to call it 'self.particles'

        ##############################################################################

        # Reset SLAM variables in case of restart
        self.num_particles = num_particles
        self.key_kp, self.key_des = None, None
        self.weight = PROB_THRESHOLD

        return estimate_pose(self.particles)

    def run(self, z, prev_kp, prev_des, kp, des):
        """
        Applies an iteration of the FastSLAM algorithm

        :param z: the new infrared height estimate for the drone
        :param prev_kp, prev_des: the keypoints and descriptors from the previous frame
        :param kp, des: the current keypoints and descriptors
        """

        # ---------------- FILE WRITING CODE DO NOT EDIT ----------------------------- #
        if WRITE_TO_FILE:
            if self.particles is not None and self.particles[0].landmarks != []:
                # write the poses of all the particles
                self.file.write(str([[p.pose[0], p.pose[1]] for p in self.particles]) + '\n')
                # write the landmark poses for the first particle
                self.file.write(str([[lm.x, lm.y] for lm in self.particles[0].landmarks]) + '\n')
                # write the GLOBAL poses of all the currently observed features relative to first particle
                poses = []
                for k in kp:
                    kp_x, kp_y = k[0], k[1]
                    # key point y is measured from the top left
                    kp_y = CAMERA_HEIGHT - kp_y

                    dx = kp_x - CAMERA_WIDTH / 2
                    dy = kp_y - CAMERA_HEIGHT / 2
                    pose = [self.pixel_to_meter(dx) + self.particles[0].pose[0],
                            self.pixel_to_meter(dy) + self.particles[0].pose[1]]
                    poses.append(pose)
                self.file.write(str(poses) + '\n')
        # ------------------------------------------------------------------------------ #

        self.z = z

        # reflect that the drone can see more or less if its height changes
        self.update_perceptual_range()

        ##################################################################################
        # TODO implement the remainder of this method
        #
        # Just as in localization, you will need to update each particle with the motion
        # estimate given by "utils.compute_transform"
        # Then, you should call self.detect_keyframe to determine whether to perform
        # a map update
        ##################################################################################

        return estimate_pose(self.particles), self.weight

    def predict_particle(self, particle, x, y, yaw):
        """
        Updates this particle's position according to the transformation from prev frame to this one.
        The robot's new position is determined based on the control, with added noise to account for control error

        :param particle:  the particle containing the robot's position information
        :param x, y, yaw: the "controls" computed by transforming the previous camera frame to this one to
                          find the dislplacement, NOTE these are measured in pixels!
        """
        ##############################################################################
        # TODO use the x,y,yaw transformation between the previous frame and this one
        #      to update the pose of particle. Be sure to add different noise for each
        #      dimension and you may set the z pose of each particle to self.z

        ##############################################################################

    def detect_keyframe(self, kp, des):
        """
        Performs a map update if 
        1) there is not a previous keyframe
        2) the distance between the previous keyframe and the current frame is above a threshold
        3) we cannot transform between the previus frame and this one. 

        :param kp, des: the lists of keypoints and descriptors
        """
        ##############################################################################
        # TODO implement this method, please call self.update_map to update the map
        #
        # Note that if transform is the transformation matrix from utils.compute_transform
        #       then: x = -transform[0, 2]
        #             y = transform[1,2]
        #             yaw = -np.arctan2(transform[1, 0], transform[0, 0])
        ##############################################################################

    def update_map(self, kp, des):
        """
        Perform a map update on every particle, sets self.weight to the average particle weight, 
        resamples the set of particles, and sets the new keyframe to the current frame.

        Please do not edit this method

        :param kp, des: the lists of keypoints and descriptors
        """
        # TODO implement this method according to the docstring
        # note: we have provided self.get_average_weight

    def update_particle(self, particle, keypoints, descriptors):
        """
        :param particle: the old particle to perform the data association on
        :param keypoints, descriptors: the lists of currently observed keypoints and descriptors

        This function is the meat of the SLAM algorithm and the main difference from your 
        localization code. Here, you will associate the currently observed features with
        the set of landmarks in each particle. You will consider each detected feature and try to 
        match it on the set of landmarks in the particle within a close range (self.perceptual_range) 
        to the drone. This ensures that the map in each particles is conditioned on the robot path
        represented by that particle alone.

        If there is a decent match (see your code from the OpenCV assignment for determining match quality),
        then you will update the ekf of that landmark with the new feature's location by calling utils.update_landmark
        and if the match is poor, then you will add the feature as a new landmark with utils.add_landmark.

        We should take adding a new landmark into account for the particle's weight since
        expanding the map reduces certainty, so subtract from the particle's weight when you do this.
        When updating a landmark, you should adjust the weight of the particle by some number proportional to the 
        quality of the match to reflect how certain we are that the observed feature matches
        the existing landmark.

        Finally, we need a mechanism to ensure that dubious, or low quality, landmarks are removed from the
        map. To do this, we will keep track of the number of times a landmark in the map is matched to by
        newly observed features, and if it not seen for a while, we will remove that landmark.

        Here, we include the function signatures for the methods from utils which you will need to 
        implement update_particle:

        def add_landmark(particle, kp, des, sigma_observation, kp_to_measurement):
            adds a newly observed landmark to particle

            :param particle: the particle to add the new landmark to
            :param kp, des: the keypoint and descriptor of the new landmark to add
            :param sigma_observation: the covariance of the observation measurement
            :param kp_to_measurement: a function which computes a range/bearing measurement from the center of the

        def update_landmark(particle, landmark, kp, des, sigma_observation, kp_to_measurement):
            update the mean and covariance of a landmark

            uses the Extended Kalman Filter (EKF) to update the existing landmark's mean (x, y) and
            covariance according to the new measurement

            :param particle: the particle to update
            :param landmark: the landmark to update
            :param kp, des: the keypoint and descriptor of the new landmark
            :param sigma_observation: the covariance of the observation measurement
            :param kp_to_measurement: a function which computes a range/bearing measurement from the center of the
                                      camera frame to kp
            :return: the updated landmark
        """

        particle.weight = PROB_THRESHOLD

        ##############################################################################
        # TODO implement this method to update the map for FastSLAM! 
        #
        # if this particle has no landmarks:
        #    add every currently observed feature as a landmark
        #    adjust particle's weight to reflect the change in certainty from adding landmarks
        # else:
        #    close landmarks = list of particle's landmarks within self.perceptual_range of the particle's pose
        #    if close landmarks is not empty:
        #        part_descriptors = list of landmark.des for each landmark in close_landmarks
        #        matched_landmarks = boolean list with same length as close_landmarks, set to false
        #
        #    for each observed feature:
        #        if part_descriptors is not empty:
        #            match = self.matcher.knnMatch(np.array([des]), np.array(part_descriptors), k=2)
        #
        #        if there was not match or the match is poor quality:
        #            add the feature as a new landmark
        #            adjust particle weight to reflect the change in certainty
        #        else:
        #            use match.trainIdx to find the index of the matching landmark in the map
        #            set the boolean corresponding to this landmark in matched_landmarks to true
        #            update the landmark in particle.landmarks with information from the newly
        #               observed feature*
        #            update the weight of the particle to reflect the quality of the match
        #               hint: use scale_weight
        #
        # * this step is the most tricky because the index of the landmark in the close_landmarks list
        #   is not the same as its index in particle.landmarks! This is because close_landmarks only
        #   contains some of the landmarks in particle.landmarks
        ##############################################################################

            # this bit of code removes dubious landmarks from the particle
            if matched_landmarks is not None:
                # increment counter for revisited particles, and decrement counter for non-revisited particles
                removed = []
                for i, match in enumerate(matched_landmarks):
                    tup = close_landmarks[i]
                    lm = particle.landmarks[tup[1]]
                    if match:
                        lm.counter += 1
                    else:
                        lm.counter -= 1
                        # 'punish' this particle for having a dubious landmark
                        particle.weight += math.log(0.1*PROB_THRESHOLD)
                        if lm.counter < 0:
                            removed.append(lm)

                for rm in removed:
                    particle.landmarks.remove(rm)

    def get_average_weight(self):
        """
        the average weight of all the particles

        :param particles: the set of particles whose weight will be averaged
        """
        return np.sum([p.weight for p in self.particles]) / float(self.num_particles)

    def pixel_to_meter(self, px):
        """
        uses the camera scale to convert pixel measurements into meters

        :param px: the distance in pixels to convert
        """
        ##############################################################################
        # TODO paste your code from localization to convert pixels to meters
        ##############################################################################

    def kp_to_measurement(self, kp):
        """
        Computes the range and bearing from the center of the camera frame to (kp.x, kp.y)
        bearing is in (-pi/2, pi/2) measured in the standard math way

        :param kp: they keypoint to measure from
        """
        kp_x, kp_y = kp[0], kp[1]
        # key point y is measured from the top left
        kp_y = CAMERA_HEIGHT - kp_y

        dx = kp_x - CAMERA_WIDTH / 2
        dy = kp_y - CAMERA_HEIGHT / 2
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        dist = self.pixel_to_meter(dist)
        bearing = math.atan2(dy, dx)

        return dist, bearing

    def update_perceptual_range(self):
        """
        computes the perceptual range of the drone: the distance from the center of the frame to the
        width-wise border of the frame
        """
        self.perceptual_range = self.pixel_to_meter(CAMERA_WIDTH / 2)

    def resample_particles(self):
        """
        resample particles according to their weight

        :param particles: the set of particles to resample
        :return: a new set of particles, resampled with replacement according to their weight
        """

        weight_sum = 0.0
        new_particles = []
        normal_weights = np.array([])

        weights = [p.weight for p in self.particles]
        lowest_weight = min(weights)

        for w in weights:
            x = 1 - (w / lowest_weight)
            if x == 0:
                x = PROB_THRESHOLD
            normal_weights = np.append(normal_weights, x)
            weight_sum += x

        normal_weights /= weight_sum
        samples = np.random.multinomial(self.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                new_particles.append(copy.deepcopy(self.particles[i]))

        self.particles = new_particles


def scale_weight(match0, match1):
    """
    uses the distances of the two best matches to provide a weight scaled between 0 and 1

    :param match0: the hamming distance of the first best match
    :param match1: the hamming distance of the second best match
    """
    scaled = (match1 - match0) / float(match1)
    if scaled == 0:
        scaled = PROB_THRESHOLD
    return scaled


def estimate_pose(particles):
    """
    retrieves the drone's estimated position by summing each particle's pose estimate multiplied
    by its weight

    Some mathematical motivation: imagine that the pose estimate of the drone along each degree of 
    freedom is represented by a random variable. We find the pose estimate by taking the expectation
    of the random variable:
    Expectation[X] = sum over all x in X of p(x) * x
    where p(x) is the weight of a particle and x is the pose of a particle

    :param particles: the set of particles to estimate a position for
    :return: the estimated pose [x,y,z,yaw]
    """

    ##############################################################################
    # TODO return the [x,y,z,yaw] pose estimate of the robot, you may reuse some or all of 
    #      your localization code to do this
    ##############################################################################


    return [x, y, z, yaw]



