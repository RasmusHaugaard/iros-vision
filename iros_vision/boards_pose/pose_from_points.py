import numpy as np
from transform3d import Transform

""" 
Use kabsch to get the pose of the taskboard and kitlayout in A's base frame.
Note that we're pairing the inner corners of the markers with the actual corners of the boards,
which are not the same points, but it should be accurate up to scale and thus give the same result.
"""

s_task = 0.384
s_kit = 0.256


def _kabsch(P, Q):  # q_t_p
    # P and Q: (N, d)
    assert P.shape == Q.shape, '{}, {}'.format(P.shape, Q.shape)
    d = P.shape[1]
    Pc, Qc = P.mean(axis=0), Q.mean(axis=0)
    P, Q = P - Pc, Q - Qc
    H = P.T @ Q
    u, _, vt = np.linalg.svd(H, full_matrices=False)
    s = np.eye(d)
    s[-1, -1] = np.linalg.det(vt.T @ u.T)
    R = vt.T @ s @ u.T
    t = Qc - R @ Pc
    return R, t


def _get_pose(Q, s):
    P = np.array([
        [0, s, 0], [s, s, 0],
        [0, 0, 0], [s, 0, 0],
    ])
    R, p = _kabsch(P, Q)
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
