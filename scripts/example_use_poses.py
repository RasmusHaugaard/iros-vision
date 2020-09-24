# scripts/example_use_poses.py
from ur_control import Robot
from transform3d import Transform
import numpy as np

r = Robot.from_ip('192.168.1.130')
q_safe = (3.317805290222168, -1.3769071859172364, -1.9077906608581543,
          -1.4512933057597657, -4.752126518880026, -1.590332333241598)

base_t_taskboard = Transform.load('base_t_taskboard')
base_t_kitlayout = Transform.load('base_t_kitlayout')

board_t_tcp_desired = Transform(p=(0, 0, 0.24), rotvec=(np.pi, 0, 0))

base_t_tcp = base_t_taskboard @ board_t_tcp_desired
# uncomment the following line to move above the kitlayout frame
# base_t_tcp = base_t_kitlayout @ board_t_tcp_desired
r.ctrl.moveJ(q_safe)
r.ctrl.moveL(base_t_tcp)
