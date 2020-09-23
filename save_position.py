import argparse

import rospy
from ur_control import Robot

parser = argparse.ArgumentParser()
parser.add_argument('--file', required=True)
args = parser.parse_args()

rospy.init_node('save_position')
r = Robot.from_ip('192.168.1.130')
r.base_t_tcp().save(args.file)
