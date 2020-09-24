import cv2
from cv2 import aruco

from ..utils import load_cam_intrinsics

K, dist_coeffs = load_cam_intrinsics('A')[:2]

params = aruco.DetectorParameters_create()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)


def _detect_markers(img):
    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters=params)

    for corner in corners:
        cv2.cornerSubPix(img, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

    if ids is None:
        return [], []
    return corners, ids


def detect_inner_corners(img):
    """we're only interested in the 'inner' corner of the markers
    (not affected by marker bending, etc)"""
    corners, ids = _detect_markers(img)
    corners = [corner[0][1] for corner in corners]
    ids = [id[0] for id in ids]
    return corners, ids


def draw_corners(img, corners, ids):
    for corner, id in zip(corners, ids):
        x, y = corner
        cv2.drawMarker(img, (x, y), (255, 0, 0), cv2.MARKER_CROSS, 10, 2)
        cv2.putText(img, str(id), (x, y), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)


def main():
    import rospy
    import matplotlib.pyplot as plt
    from .cam import get_img

    rospy.init_node('detector_node')

    img = get_img()
    corners, ids = detect_inner_corners(img)
    draw_corners(img, corners, ids)
    plt.imshow(img)
    plt.show()


if __name__ == '__main__':
    main()
