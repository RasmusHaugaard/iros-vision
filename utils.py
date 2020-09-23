import numpy as np


def kabsch(P, Q):  # q_t_p
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
