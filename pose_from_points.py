import numpy as np
from utils import kabsch
from transform3d import Transform

s_task = 0.384
s_kit = 0.256


def _get_pose(Q, s):
    P = np.array([
        [0, s, 0], [s, s, 0],
        [0, 0, 0], [s, 0, 0],
    ])
    R, p = kabsch(P, Q)
    return Transform(R=R, p=p)


def get_poses(pts_base):
    pts_task_base = pts_base[:4]  # (4, 3)
    pts_kit_base = pts_base[4:]  # (4, 3)

    base_t_taskboard = _get_pose(pts_task_base, s_task)
    base_t_kitlayout = _get_pose(pts_kit_base, s_kit)

    return base_t_taskboard, base_t_kitlayout


def main():
    pts_base = np.load('points_base.npy')
    base_t_taskboard, base_t_kitlayout = get_poses(pts_base)
    print(base_t_taskboard)
    print(base_t_kitlayout)


if __name__ == '__main__':
    main()
