import time
import argparse

from ur_control import Robot
from transform3d import Transform

from gripper import get_gripper

parser = argparse.ArgumentParser()
parser.add_argument('obj_names', nargs='+')
parser.add_argument('--wait', type=float, default=1.)
parser.add_argument('--dont-put-back', action='store_true')
parser.add_argument('--stop-at-grasp', action='store_true')
args = parser.parse_args()

obj_names = args.obj_names

gripper = get_gripper()
gripper.open()

r = Robot.from_ip('192.168.1.130')
r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
q_safe = (2.8662071228027344, -1.7563158474364222, -1.9528794288635254,
          -1.0198443692973633, -4.752078358327047, -2.1280840078936976)

grasp_start_widths = {
    'big_round_peg': 25,
    'small_round_peg': 17,
    'big_gear': 40,
    'small_gear': 26,
    'bnc': 24,
    'belt': 30,
    'small_square_peg': 14,
    'big_square_peg': 22,
    'ethernet_cable_head': 22,
}

for obj_name in obj_names:
    r.ctrl.moveJ(q_safe)

    base_t_kit = Transform.load('base_t_kitlayout')
    kit_t_obj = Transform.load(f'kit_t_objects_practice/kit_t_{obj_name}')
    tcp_t_obj_grasp = Transform.load(f'tcp_t_obj_grasp/tcp_t_{obj_name}_grasp')
    base_t_tcp_grasp = base_t_kit @ kit_t_obj @ tcp_t_obj_grasp.inv

    r.ctrl.moveL(base_t_tcp_grasp @ Transform(p=(0, 0, -0.05)))
    gripper.move(grasp_start_widths[obj_name], 255, 255)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.grasp(0, 0, 100, 0)
    if args.stop_at_grasp:
        quit()
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
    if args.dont_put_back:
        quit()
    time.sleep(args.wait)
    r.ctrl.moveL(base_t_tcp_grasp)
    gripper.move(grasp_start_widths[obj_name], 255, 255)
    r.ctrl.moveL(r.base_t_tcp() @ Transform(p=(0, 0, -0.05)))
