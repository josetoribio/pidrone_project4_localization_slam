"""
compute_displacement

In this task, you will use OpenCV to implement the motion
model for localization and SLAM on the drone. This involves computing the
distance between overlapping camera frames taken by the drone as it flies.

We reccomend completing the assignment on a department machine as they have
OpenCV and NumPy installed.

In this task, you will be expected to read OpenCV documentation in order to
use various the functions properly. Please visit https://docs.opencv.org/2.4.9/
(2.4.9 is the version of OpenCV on the drone) and use the search bar to locate
the indicated functions.

*****IMPORTANT NOTE*****
Any time you see a sentence end in a question mark in this document, know that
they are NOT rhetorical questions. PLEASE write an answer in answers.txt, the
questions will help you, we promise. It is not necessary to use complete sentences.
************************
"""

import cv2
import numpy as np
import math

image_A = cv2.imread('img_A.jpg') 
image_B = cv2.imread('img_B.jpg') 

# Step One: Extract Features
NUM_FEATURES = <choose a number of features>
detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
# TODO search "ORB" in the documentation, use detector.detectAndCompute, called
# ORB::Operator() in the documentation, to extract kp and des from both images

# Hint: kp are keypoints, des are descriptors. Remember that each feature is
# described by a kp and a des

# Hint 2: the mask argument should be None

###############################################################################

# Step Two: Match Features and Descriptors
index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)
matcher = cv2.FlannBasedMatcher(index_params, search_params)
# TODO search "Descriptor Matcher" in the documentation, use knnMatch to obtain
# the list of matches. How is the list of matches formatted? What kind of objects
# are in the list? What fields do they have? Decide on a value of k.
# Why did you pick that value?

# TODO filter the matches based on their quality.

# Hint: a large distance between descriptors means a poor match, and a small
#       distance indicates a strong match

# Hint: knnMatch finds the k best matches in the second input list for each
# descriptor in the first input list. Compare the best matches for each descriptor
# to judge the quality of a match. Consider the case when the best match is
# similar in quality to the second best match, or when it is much better. What
# will that tell you about the quality of the match between the descriptors
# used to compute the first match?


# TODO Obtain lists of the keypoint coordinates from both image A and image B
# for each of the "good" matches from. Search "Keypoint" in the documentation to find 
# details on the keypoint object. What field does it have to hold feature coordinates?


###############################################################################

# Step Three: Estimate Transform
imageA_points = np.float32(<list of image A filtered keypoints>).reshape(-1, 1, 2)
imageB_points = np.float32(<list of image B filtered keypoints>).reshape(-1, 1, 2)
# TODO search "estimate transform" in the documentation, use estimateRigidTransform
# to find the transformation between imageA_points and imageB_points

# Hint: 3rd argument to esimateRigidTransform should be False

###############################################################################

# Step Four: Convert to Meters
transform = <transformation computed above>
x_displacement = -transform[0, 2]
y_displacement = transform[1, 2]
yaw_displacement = -np.arctan2(transform[1, 0], transform[0, 0])

# TODO use the height of the drone to convert the displacement to meters. To do
# this, we multiply the height (meters) by a specific camera scale to obtain
# the meter per pixel value in the camera frame for that specific height. Then,
# we can multiply this ratio by the pixel displacement to find the meter
# displacement. What unit must this camera scale have? How might you
# experimentally determine the camera scale for the drone's camera? Bonus
# question (not graded), how might you mathematically determine the scale?

# The camera scale is 290
# The height of the drone is 0.23m


# TODO finally, compute the meter distance between the two image frames

###############################################################################
