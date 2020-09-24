import time

import rospy
from sensor_msgs.msg import Image as ImageMsg
from ros_numpy.image import image_to_numpy


def get_img(exposure=1 / 30):
    """the exposure parameter is just to avoid blur. It does not change the actual exposure"""
    now = time.time()
    while True:
        msg = rospy.wait_for_message('/camera_A/image_rect', ImageMsg)  # type: ImageMsg
        if msg.header.stamp.to_sec() - exposure > now:
            break
    return image_to_numpy(msg)
