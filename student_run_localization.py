#!/usr/bin/env python
"""
vision_localization_onboard.py

This file can run SLAM or localization on board, offline or online
"""


from student_localization_helper import *
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import picamera
import camera_info_manager
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Range, CameraInfo
from analyze_flow import AnalyzeFlow
from std_msgs.msg import Empty
from pidrone_pkg.msg import State
import argparse
import tf

# ---------- map parameters ----------- #
MAP_PIXEL_WIDTH = 3227  # in pixel
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4  # in meter
MAP_REAL_HEIGHT = 1.07

# ---------- camera parameters ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# assume a pixel in x and y has the same length
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)

# ---------- localization parameters ----------- #
MAX_BAD_COUNT = -10
NUM_PARTICLE = 30
NUM_FEATURES = 200


class Localizer(picamera.array.PiMotionAnalysis):

    def __init__(self, camera, bridge):
        picamera.array.PiMotionAnalysis.__init__(self, camera)
        self.bridge = bridge
        self.br = tf.TransformBroadcaster()

        self.posepub = rospy.Publisher('/pidrone/picamera/pose', PoseStamped, queue_size=1)
        self.first_image_pub = rospy.Publisher("/pidrone/picamera/first_image", Image, queue_size=1, latch=True)

        self.detector = cv2.ORB(nfeatures=NUM_FEATURES, scoreType=cv2.ORB_FAST_SCORE)

        map_grid_kp, map_grid_des = create_map('map.jpg')
        self.estimator = LocalizationParticleFilter(map_grid_kp, map_grid_des)

        # [x, y, z, yaw]
        self.pos = [0, 0, 0, 0]
        self.posemsg = PoseStamped()

        self.angle_x = 0.0
        self.angle_y = 0.0
        # localization does not estimate z
        self.z = 0.0

        self.first_locate = True
        self.locate_position = False

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None
        self.prev_rostime = None

        self.map_counter = 0
        self.max_map_counter = 0

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    def write(self, data):
        curr_img = np.reshape(np.fromstring(data, dtype=np.uint8), (CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        curr_rostime = rospy.Time.now()
        self.posemsg.header.stamp = curr_rostime
        curr_time = curr_rostime.to_sec()

        # start MCL localization
        if self.locate_position:
            curr_kp, curr_des = self.detector.detectAndCompute(curr_img, None)

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    particle = self.estimator.initialize_particles(NUM_PARTICLE, curr_kp, curr_des)
                    self.first_locate = False
                    self.pos = [particle.x(), particle.y(), particle.z(), particle.yaw()]

                    self.posemsg.pose.position.x = particle.x()
                    self.posemsg.pose.position.y = particle.y()
                    self.posemsg.pose.position.z = particle.z()
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                    self.posemsg.pose.orientation.x = x
                    self.posemsg.pose.orientation.y = y
                    self.posemsg.pose.orientation.z = z
                    self.posemsg.pose.orientation.w = w

                    self.posepub.publish(self.posemsg)
                    print 'first', particle
                else:
                    particle = self.estimator.update(self.z, self.angle_x, self.angle_y, self.prev_kp, self.prev_des,
                                                     curr_kp, curr_des)

                    # update position
                    self.pos = [self.hybrid_alpha * particle.x() + (1.0 - self.hybrid_alpha) * self.pos[0],
                                self.hybrid_alpha * particle.y() + (1.0 - self.hybrid_alpha) * self.pos[1],
                                self.z,
                                self.alpha_yaw * particle.yaw() + (1.0 - self.alpha_yaw) * self.pos[3]]

                    self.posemsg.pose.position.x = self.pos[0]
                    self.posemsg.pose.position.y = self.pos[1]
                    self.posemsg.pose.position.z = self.pos[2]
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, self.pos[3])

                    self.posemsg.pose.orientation.x = x
                    self.posemsg.pose.orientation.y = y
                    self.posemsg.pose.orientation.z = z
                    self.posemsg.pose.orientation.w = w

                    self.posepub.publish(self.posemsg)
                    print '--pose', self.pos[0], self.pos[1], self.pos[3]

                    # if all particles are not good estimations
                    if is_almost_equal(particle.weight(), PROB_THRESHOLD):
                        self.map_counter = self.map_counter - 1
                    elif self.map_counter <= 0:
                        self.map_counter = 1
                    else:
                        self.map_counter = min(self.map_counter + 1, -MAX_BAD_COUNT)

                    # if it's been a while without a significant average weight
                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.map_counter = 0
                        print 'Restart localization'

                    print 'count', self.map_counter
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.pos[3]),
                              rospy.Time.now(),
                              "base",
                              "world")

    def state_callback(self, data):
        """ update z, angle x, and angle y data when /pidrone/state is published to """
        self.z = data.pose_with_covariance.pose.position.z
        self.angle_x = data.twist_with_covariance.twist.angular.x
        self.angle_y = data.twist_with_covariance.twist.angular.y

    def reset_callback(self, data):
        """start localization when '/pidrone/reset_transform' is published to (press 'r')"""
        print "Start localization"
        self.locate_position = True
        self.first_locate = True
        self.map_counter = 0
        self.max_map_counter = 0


def is_almost_equal(x, y):
    epsilon = 1e-8
    return abs(x-y) <= epsilon


def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)

    image_pub = rospy.Publisher("/pidrone/picamera/image_raw", Image, queue_size=1, tcp_nodelay=False)
    camera_info_pub = rospy.Publisher("/pidrone/picamera/camera_info", CameraInfo, queue_size=1, tcp_nodelay=False)

    cim = camera_info_manager.CameraInfoManager("picamera", "package://pidrone_pkg/params/picamera.yaml")
    cim.loadCameraInfo()
    if not cim.isCalibrated():
        rospy.logerr("warning, could not find calibration for the camera.")

    try:
        bridge = CvBridge()

        with picamera.PiCamera(framerate=90) as camera:
            camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
            with AnalyzeFlow(camera) as flow_analyzer:
                flow_analyzer.setup(camera.resolution)
                phase_analyzer = Localizer(camera, bridge)

                rospy.Subscriber('/pidrone/reset_transform', Empty, phase_analyzer.reset_callback)
                rospy.Subscriber('/pidrone/state', State, phase_analyzer.state_callback)

                camera.start_recording("/dev/null", format='h264', splitter_port=1, motion_output=flow_analyzer)
                print "Starting Flow"
                camera.start_recording(phase_analyzer, format='bgr', splitter_port=2)
                last_time = None
                while not rospy.is_shutdown():
                    camera.wait_recording(1 / 100.0)

                    if phase_analyzer.prev_img is not None and phase_analyzer.prev_time != last_time:
                        image_message = bridge.cv2_to_imgmsg(phase_analyzer.prev_img, encoding="bgr8")
                        image_message.header.stamp = phase_analyzer.prev_rostime
                        last_time = phase_analyzer.prev_rostime
                        image_pub.publish(image_message)
                        camera_info_pub.publish(cim.getCameraInfo())

                camera.stop_recording(splitter_port=1)
                camera.stop_recording(splitter_port=2)
        print "Shutdown Received"
    except Exception:
        print "Camera Error!!"
        raise


if __name__ == '__main__':
    main()
