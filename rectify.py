import json

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image as ImageMsg
from ros_numpy.image import image_to_numpy, numpy_to_image

rospy.init_node('rectifier_node')
pub_image_rect = rospy.Publisher('/pylon_camera_node/image_rect', ImageMsg, queue_size=1)

calib = json.load(open('camera_intrinsics.json'))
K = np.array(calib['camera_matrix'])
dist_coeffs = np.array(calib['dist_coeffs'])
h, w = calib['h'], calib['w']

undistort_maps = cv2.initUndistortRectifyMap(K, dist_coeffs, np.eye(3), K, (w, h), cv2.CV_32FC1)


def cb(in_msg: ImageMsg):
    img = image_to_numpy(in_msg)
    img = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)
    rect = cv2.remap(img, *undistort_maps, cv2.INTER_LINEAR)
    out_msg = numpy_to_image(rect, 'rgb8')
    out_msg.header.stamp = in_msg.header.stamp
    pub_image_rect.publish(out_msg)


sub = rospy.Subscriber("/pylon_camera_node/image_raw", ImageMsg, cb, queue_size=1)

rospy.spin()
