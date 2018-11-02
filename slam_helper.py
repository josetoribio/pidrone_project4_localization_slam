"""
student_slam_helper.py

Here is where you will implement FastSLAM for the PiDrone. Follow the docstrings
and the directions in the online text book to complete the assignment. When you
are ready to test your code, run the included slam.py file, which will read
the sample flight data from map_data.txt and run your SLAM implementation.
The pre-implemented code in the "if POSE" block in the run method below will
write the output of your program to pose_data.txt. You can run animate_slam.py
to view the animation of your program! It should resemble the sample animation
which we provide.
"""

import numpy as np
import math
import utils
import copy
import cv2

# set to true in order to print the poses to a file
POSE  = True

# ----- camera parameters DO NOT EDIT ------- #
CAMERA_SCALE = 290.
CAMERA_WIDTH = 320.
CAMERA_HEIGHT = 240.
MATCH_RATIO = 0.7

# ----- SLAM parameters ------- #
PROB_THRESHOLD = 0.005
KEYFRAME_DIST_THRESHOLD = # TODO choose this value
KEYFRAME_YAW_THRESHOLD = # TODO choose this value

# ----- edit to where you want the pose data written --------- #
path = 'pose_data.txt'


class Particle:
    """
    attributes:
    robot_position: a list of the robot's position (x, y, z, yaw)
    landmarks:      a list of landmark objects
    descriptors:    a list of feature descriptors for each landmark
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

        if POSE:
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
        Each particle starts at (0,0) since we build the map relative to the drone's
        initial position, but with some noise

        :param num_particles: the number of particles to generate
        """
        self.particles = [Particle(abs(utils.normal(0, 0.1)),
                                   abs(utils.normal(0, 0.1)),
                                   self.z,
                                   abs(utils.normal(math.pi, 0.01))) for _ in range(num_particles)]

        # Reset SLAM variables in case of restart
        self.num_particles = num_particles
        self.key_kp, self.key_des = None, None
        self.weight = PROB_THRESHOLD

        return estimate_pose(self.particles)

    def run(self, z, prev_kp, prev_des, kp, des):
        """
        applies an iteration of the FastSLAM algorithm

        :param z: the new infrared height estimate for the drone
        :param prev_kp, prev_des: are the keypoints and descriptors from the previous frame
        :param kp, des: the current keypoints and descriptors
        """

        # print the average number of landmarks per particles
        # print "LM: ", np.sum([len(p.landmarks) for p in self.particles]) / float(self.num_particles)

        # so the issue now is that the kp are local and everything else global, but we can
        # put the kp through kp_to_measurement and just add the range/bearing to particle[0] pose

        # or can I literally just do particle pose [x,y] + [dx , dy] and that gives the global pose question mark
        # yes that is true let's do that

        # write data to a text file to be animated
        if POSE:
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

        self.z = z

        # reflect that the drone can see more or less if its height changes
        self.update_perceptual_range()

        # compute transformation from previous frame
        transform = utils.compute_transform(self.matcher, prev_kp, prev_des, kp, des)

        if transform is not None:
            x = -transform[0, 2]
            y = transform[1, 2]
            yaw = -np.arctan2(transform[1, 0], transform[0, 0])

            # update poses with motion prediction
            for p in self.particles:
                self.predict_particle(p, x, y, yaw)

            # (potentially) do a map update
            self.detect_keyframe(kp, des)

        return estimate_pose(self.particles), self.weight

    def predict_particle(self, particle, x, y, yaw):
        """
        updates this particle's position according to the transformation from prev frame to this one

        the robot's new position is determined based on the control, with
        added noise to account for control error

        :param particle:  the particle containing the robot's position information
        :param x, y, yaw: the "controls" computed by transforming the previous camera frame to this one
        """

        noisy_x_y_z_yaw = np.random.multivariate_normal([x, y, self.z, yaw], self.covariance_motion)

        particle.pose[0] += self.pixel_to_meter(noisy_x_y_z_yaw[0])
        particle.pose[1] += self.pixel_to_meter(noisy_x_y_z_yaw[1])
        particle.pose[2] = self.z
        particle.pose[3] += noisy_x_y_z_yaw[3]
        particle.pose[3] = utils.adjust_angle(particle.pose[3])

    def detect_keyframe(self, kp, des):
        """
        Checks if there is a previous keyframe, and if not, starts  a new one. If the distance between the
        previous keyframe and the current frame is above a threshold, starts a map update thread. Or, if
        we cannot transform between the previous keyframe and this frame, also starts a map update thread.

        :param kp, des: the lists of keypoints and descriptors
        """
        # there is some previous keyframe
        if self.key_kp is not None and self.key_des is not None:

            transform = utils.compute_transform(self.matcher, self.key_kp, self.key_des, kp, des)

            if transform is not None:
                # distance since previous keyframe
                x = self.pixel_to_meter(-transform[0, 2])
                y = self.pixel_to_meter(transform[1, 2])
                yaw = -np.arctan2(transform[1, 0], transform[0, 0])

                if utils.distance(x, y, 0, 0) > self.pixel_to_meter(KEYFRAME_DIST_THRESHOLD) \
                        or yaw > KEYFRAME_YAW_THRESHOLD:
                    self.update_map(kp, des)
            else:
                # moved too far to transform from last keyframe, so set a new one
                self.update_map(kp, des)
        # there is no previous keyframe
        else:
            self.update_map(kp, des)

    def update_map(self, kp, des):
        """
        DOESN'T start a thread to update the map

        :param kp, des: the lists of keypoints and descriptors
        """
        for p in self.particles:
            self.update_particle(p, kp, des)

        self.weight = self.get_average_weight()
        self.resample_particles()

        # set the new keyframe kp and des to the current ones
        self.key_kp, self.key_des = kp, des

    def update_particle(self, particle, keypoints, descriptors):
        """
        Associate observed keypoints with an old particle's landmark set and update the EKF
        Increment the landmark's counter if it finds a match, otherwise add a new landmark
        Decrement the counter of a landmark which is close to this particle's pose and not observed

        :param particle: the old particle to perform the data association on
        :param keypoints, descriptors: the lists of currently observed keypoints and descriptors
        """

        particle.weight = PROB_THRESHOLD

        # if this particle has no landmarks, make all measurements into landmarks
        if len(particle.landmarks) == 0:
            for kp, des in zip(keypoints, descriptors):
                utils.add_landmark(particle, kp, des, self.sigma_observation, self.kp_to_measurement)
                particle.weight += math.log(PROB_THRESHOLD)
        else:
            # find particle's landmarks in a close range, close_landmarks holds tuples of the landmark and its index
            close_landmarks = []
            for i, lm in enumerate(particle.landmarks):
                if utils.distance(lm.x, lm.y, particle.pose[0], particle.pose[1]) <= self.perceptual_range * 1.2:
                    close_landmarks.append((lm, i))

            part_descriptors, matched_landmarks = None, None
            if len(close_landmarks) != 0:
                # get the descriptors of relevant landmarks
                part_descriptors = [lm[0].des for lm in close_landmarks]

                # we will set to true indices where a landmark is matched
                matched_landmarks = [False] * len(close_landmarks)

            for kp, des in zip(keypoints, descriptors):
                # length 1 list of the most likely match between this descriptor and all the particle's descriptors
                match = None
                if part_descriptors is not None:
                    match = self.matcher.knnMatch(np.array([des]), np.array(part_descriptors), k=2)

                # there was no match (short circuiting!)
                if match is None or len(match) < 2 or match[0].distance > MATCH_RATIO * match[1].distance:
                    utils.add_landmark(particle, kp, des, self.sigma_observation, self.kp_to_measurement)

                    # 'punish' this particle since new landmarks decrease certainty
                    particle.weight += math.log(PROB_THRESHOLD)
                else:
                    # get the index of the matched landmark in close_landmarks
                    close_index = match[0].trainIdx
                    matched_landmarks[close_index] = True
                    dated_landmark = close_landmarks[close_index]

                    # update the original landmark in this particle
                    updated_landmark = utils.update_landmark(particle, dated_landmark[0], kp, des,
                                                             self.sigma_observation, self.kp_to_measurement)
                    particle.landmarks[dated_landmark[1]] = updated_landmark

                    # 'reward' this particles since revisiting landmarks increases certainty
                    particle.weight += math.log(scale_weight(match[0].distance, match[1].distance))

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
        Gets the average weight of all the particles in the filter

        :param: none
        :return: the average weight of all the particles in the filter
        """
        # TODO implement this method (try to do it in one line!)

    def pixel_to_meter(self, px):
        """
        Uses the camera scale to convert pixel measurements into meters. You
        should use your code from the localization assignment.

        :param px: the distance in pixels to convert
        :return: px converted to meters
        """
        return px * self.z / CAMERA_SCALE

    def kp_to_measurement(self, kp):
        """
        Computes the range and bearing from the center of the camera frame to
        kp.pt, which is the [x,y] location of an extracted feature. The bearing
        should be in (-pi/2, pi/2) measured in the standard math way.

        :param kp: they keypoint to measure from
        :return: the distance, bearing from the center of the camera to kp

        You do not need to edit this method.
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
        Resets the perceptual range of the drone which indicates how far the
        drone can "see." The distance is from the point in the plane directly
        below the drone to the farthest point in the plane it can see. You s
        should decide on a reasonable proxy for the perceptual range. This
        method must be called in every time step because the drone can see
        farther as it flies higher!
        """
        # TODO impement this method

    def resample_particles(self):
        """
        Resamples the particles according to their weight. This method should be
        largely the same as the version from localization, except you will be
        replacing the self.particles list with one made by deep copying from the
        existing set of particles.

        :param: nothing
        :return: nothing
        """

        # TODO implement this method


def scale_weight(match0, match1):
    """
    Uses the Hamming distances of the two best matches for a descriptor to
    provide a weight scaled between 0 and 1 which represents the probability of
    the match. Consider how you can use the difference in Hamming distance
    (match quality) between the two best matches for a descriptor to measure how
    "good" the first match is.

    :param match0: the hamming distance of the first best match
    :param match1: the hamming distance of the second best match
    :return: the scaled weight which indicates the quality of the match
    """
    # TODO implement this method


def estimate_pose(particles):
    """
    Return the drone's estimated position using the expectation of the
    belief distribution, which is the weighted sum of the poses of all the
    particles in the filter, weighted by the weight of the particle. Note
    that the weights must be normalized to 1 for this to work. This code should
    be the same as in localization.

    Some mathematical motivation Expectation[X] =
                                                sum over all x in X of p(x) * x

    :param particles: the set of particles to estimate a position for
    :return: the estimated pose [x,y,z,yaw]
    """
    # TODO implement this method
