import json
from pathlib import Path

import numpy as np
from transform3d import Transform

calib_folder = Path(__file__).parent / 'calib'


def load_cam_intrinsics(name):
    calib = json.load((calib_folder / name / 'camera_intrinsics.json').open())
    K = np.array(calib['camera_matrix'])
    dist_coeffs = np.array(calib['dist_coeffs'])
    return K, dist_coeffs


def load_tcp_t_cam(name):
    return Transform.load(calib_folder / name / 'tcp_t_cam')


def load_table_t_base(name):
    return Transform.load(calib_folder / name / 'table_t_base')
