from ur_control import Robot
from transform3d import Transform
import rospy

from .sweep import sweep
from .sweep_reconstruction import reconstruct_points
from .pose_from_points import get_poses
from ..utils import load_cam_intrinsics


def capture_images_and_find_boards(r: Robot):
    images, base_t_tcps = sweep(r, 40)
    pts_base = reconstruct_points(
        images=images, base_t_tcps=base_t_tcps,
        K=load_cam_intrinsics('A')[0], tcp_t_cam=Transform.load('tcp_t_cam')
    )
    base_t_taskboard, base_t_kitlayout = get_poses(pts_base)
    return base_t_taskboard, base_t_kitlayout


def main():
    rospy.init_node('iros_vision_main_node')
    r = Robot.from_ip('192.168.1.130')
    base_t_taskboard, base_t_kitlayout = capture_images_and_find_boards(r)
    print('base_t_taskboard', base_t_taskboard)
    print('base_t_kitlayout', base_t_kitlayout)


if __name__ == '__main__':
    main()
