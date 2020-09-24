# scripts/example_get_poses.py
import iros_vision
from ur_control import Robot
import rospy

rospy.init_node('example_node')  # needed to take images
r = Robot.from_ip('192.168.1.130')
# or: r = Robot(rtde_r, rtde_c)

q_safe = (3.317805290222168, -1.3769071859172364, -1.9077906608581543,
          -1.4512933057597657, -4.752126518880026, -1.590332333241598)
# The below function moves robot A above the workspace.
# the other robots must be clear of the workspace

r.ctrl.moveJ(q_safe)
base_t_taskboard, base_t_kitlayout = iros_vision.capture_images_and_find_boards(r)
r.ctrl.moveJ(q_safe)

base_t_taskboard.save('base_t_taskboard')
base_t_kitlayout.save('base_t_kitlayout')
