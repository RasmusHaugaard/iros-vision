from typing import List

import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm
from transform3d import Transform

from .detect_markers import detect_inner_corners, draw_corners


def reconstruct_points(images: List[np.ndarray], base_t_tcps: List[Transform],
                       tcp_t_cam: Transform, K: np.ndarray, debug=False):
    detections = [detect_inner_corners(img) for img in tqdm(images)]

    n_times_detected = np.zeros(8)
    for _, ids in detections:
        for i in ids:
            n_times_detected[i] += 1
    print('Number of detections per marker:')
    print(n_times_detected)
    assert np.all(n_times_detected >= 3), 'at least 3 detections per marker'

    cam_t_bases = [(base_t_tcp @ tcp_t_cam).inv.matrix for base_t_tcp in base_t_tcps]
    cam_t_bases = torch.tensor(np.stack(cam_t_bases))  # (n, 4, 4)
    cam_R_bases = cam_t_bases[:, :3, :3]  # (n, 3, 3)
    cam_p_bases = cam_t_bases[:, :3, 3:4]  # (n, 3, 1)

    K = torch.tensor(K)  # (3, 3)

    points_base = torch.zeros((3, 8), dtype=torch.double, requires_grad=True)

    for lr in 1e-2, 1e-3:
        opt = torch.optim.Adam([points_base], lr=lr)
        pbar = tqdm(range(300))
        for _ in pbar:
            points_cam = cam_R_bases @ points_base + cam_p_bases  # (n, 3, 8)
            points_img = K @ points_cam
            points_img = points_img[:, :2] / points_img[:, 2:3]  # (n, 2, 8)

            losses = []
            dists = []
            for img_i, (corners, ids) in zip(range(len(images)), detections):
                for corner, marker_i in zip(corners, ids):
                    corner = torch.tensor(corner)
                    dist = torch.norm(points_img[img_i, :, marker_i] - corner)
                    dists.append(dist.item())
                    # TODO: square dist
                    losses.append(dist / n_times_detected[marker_i])

            loss = torch.sum(torch.stack(losses))

            opt.zero_grad()
            loss.backward()
            opt.step()
            pbar.set_description(f'{np.mean(dists):.2f}')

    points_base = points_base.detach().numpy().T

    if debug:
        rows = cols = int(np.ceil(np.sqrt(len(images))))
        fig, axs = plt.subplots(rows, cols)
        points_img = points_img.detach().numpy().transpose(0, 2, 1)  # (n, 8, 2)
        for img, ax, points in zip(images, axs.reshape(-1), points_img):
            points = np.around(points).astype(int)
            img = img.copy()
            draw_corners(img, points, list(range(8)))
            ax.imshow(img)
        for ax in axs.reshape(-1):
            ax.axis('off')
        plt.show()

    return points_base


def main():
    import time
    from ..utils import load_tcp_t_cam, load_cam_intrinsics

    n = 40
    images = [cv2.imread(f'sweep/{i}.png') for i in tqdm(range(n))]
    base_t_tcps = [Transform.load(f'sweep/base_t_tcp_{i}') for i in tqdm(range(n))]
    tcp_t_cam = load_tcp_t_cam('A')
    K, _ = load_cam_intrinsics('A')

    start = time.time()
    points_base = reconstruct_points(images=images, base_t_tcps=base_t_tcps,
                                     tcp_t_cam=tcp_t_cam, K=K, debug=False)
    print(f'{time.time() - start:.2f} s')

    print(points_base)
    np.save('points_base.npy', points_base)


if __name__ == '__main__':
    main()
