from ur_control import Robot
from transform3d import Transform
import rospy

from sweep import sweep
from sweep_reconstruction import reconstruct_points
from detect_boards import K
from pose_from_points import get_poses

rospy.init_node('usage_node')

r = Robot.from_ip('192.168.1.130')
images, base_t_tcps = sweep(r, 40)
pts_base = reconstruct_points(
    images=images, base_t_tcps=base_t_tcps,
    K=K, tcp_t_cam=Transform.load('tcp_t_cam')
)
base_t_taskboard, base_t_kitlayout = get_poses(pts_base)

print('base_t_taskboard', base_t_taskboard)
print('base_t_kitlayout', base_t_kitlayout)
