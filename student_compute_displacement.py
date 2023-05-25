"""
compute_displacement

In this task, you will use OpenCV to implement the motion
model for localization and SLAM on the drone. This involves computing the
distance between overlapping camera frames taken by the drone as it flies.

We reccomend completing the assignment on a department machine as they have
OpenCV and NumPy installed, installing these dependencies yourself, or
even solving this on your drone!

In this task, you will be expected to read OpenCV documentation in order to
use various the functions properly. Please visit https://docs.opencv.org/2.4.9/
(2.4.9 is the version of OpenCV on the drone) and use the search bar to locate
the indicated functions.
"""

import cv2
import numpy as np
import math

image_A = cv2.imread('img_A.jpg') 
image_B = cv2.imread('img_B.jpg') 

############################################################################### 
# Step One: Extract Features
# In this step, we extract the coordinates of salient features from each image,
# called keypoints, as well as descriptors of those features.

NUM_FEATURES = ... #<choose a number of features>
detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)

# TODO search "ORB" in the documentation, use detector.detectAndCompute(), called
# ORB::Operator() in the documentation, to extract keypoints and descriptors
# from both images

# Hint: kp are keypoints, des are descriptors. Remember that each feature is
# described by a kp and a des.

# Hint 2: you may use None as the mask argument

###############################################################################
# Step Two: Match Features and Descriptors
# In this step, we compare the descriptors of features in each image to
# to find the features in each image which describe the same point.

index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)
matcher = cv2.FlannBasedMatcher(index_params, search_params)

# TODO Search "Descriptor Matcher" in the documentation.
# Answer the following in answers.txt:
# How is the list of matches formatted? 
# What kind of objects are in the list? 
# What fields do they have? 
# Decide on a value of k. Why did you pick that value? It may 
# be helpful to the rest of the comments in this step before 
# answering this question!

# TODO use knnMatch to obtain the list of matches between the image_A 
# descriptors and the image_B descriptors

# Now, we will filter out the poor matches from the matches list

# Hint: a large distance between descriptors means a poor match, and a small
# distance indicates a strong match

# Hint 2: knnMatch finds the k best matches in the second input list for each
# descriptor in the first input list. Compare the best matches for each descriptor
# to judge the quality of a match. What does it mean when the best match is
# similar in quality to the second best match? What does it mean when the best
# match is much better than the second best match? 

# TODO filter the matches based on their quality.

# Now, we will recover the coordinates in each image of the 
# features we have just matched. Each match corresponds to two features: one in
# image_A, and one in image_B, which both describe the same point.

# TODO Answer the following in answers.txt:
# Search "Keypoint" in the documentation to find 
# details on the keypoint object. 
# What field does the keypoint object have to hold feature coordinates?

# TODO Obtain lists of the keypoint coordinates from both image A and image B
# for each of the filtered matches. 

image_A_match_keypoints = ... #<list of image_A kp coords for each match>
image_B_match_keypoints = ... #<list of image_B kp coords for each match>


###############################################################################
# Step Three: Estimate Transform
# In this step, we will use the lists of coordinates of the matching features
# from each image to compute a transformation between the two pictures.

imageA_points = np.float32(image_A_match_keypoints).reshape(-1, 1, 2)
imageB_points = np.float32(image_B_match_keypoints).reshape(-1, 1, 2)

# TODO search "estimate transform" in the documentation, use estimateRigidTransform
# to find the transformation between imageA_points and imageB_points

# Hint: 3rd argument to esimateRigidTransform should be False

###############################################################################
# Step Four: Convert to Meters
# In this step we will use the height of the drone to convert the pixel displacement 
# from the transformation to meters.

transform = ... #<transformation computed with estimateRigidTransform>
x_displacement = -transform[0, 2]
y_displacement = transform[1, 2]
yaw_displacement = -np.arctan2(transform[1, 0], transform[0, 0])

# To do this, we multiply the height (meters) by a specific camera scale to obtain
# the meter-per-pixel value of the camera. This number varies with the height
# of the drone. Then, we can multiply this ratio by the pixel displacement (what you
# just computed) to find the meter displacement. 

# TODO answer the following in answers.txt:
# What unit must the camera scale have? 
# How might you experimentally determine the camera scale for the drone's camera? 
# Bonus question:how might you mathematically determine the scale?

# The camera scale is 290
# The height of the drone is 0.23m

# TODO compute the meter distance between the two image frames and write 
# your answer in answers.txt

###############################################################################
