import numpy as np
import rospy
from ur_control import Robot
from transform3d import Transform

from .cam import get_img

x0, x1 = -0.4, -0.8
y0, y1 = -0.4, 0.7
z = 0.5
look_angle = np.deg2rad(15)


def sweep(r: Robot, n: int):
    """Moves robot A in a U-shape over the workspace collecting n images"""
    s = np.linspace(0, 1, n)
    s0, s1 = s[::2], s[1::2]

    images, base_t_tcps = [], []

    for start, end, ss, d in [((x0, y0), (x0, y1), s0, 1),
                              ((x1, y1), (x1, y0), s1, -1)]:
        angle_axis = 0, -np.pi + d * look_angle, 0
        start = Transform.from_xyz_rotvec([*start, z, *angle_axis])
        end = Transform.from_xyz_rotvec([*end, z, *angle_axis])

        for s in ss:
            base_t_tcp = start.lerp(end, s)
            r.ctrl.moveL(base_t_tcp)
            img = get_img()

            images.append(img)
            base_t_tcps.append(r.base_t_tcp())
    return images, base_t_tcps


def main():
    import time
    import cv2

    rospy.init_node('sweep')
    r = Robot.from_ip('192.168.1.130')

    start = time.time()
    images, base_t_tcps = sweep(r, n=40)
    print(f'{time.time() - start:.2f} s')

    for i, (img, base_t_tcp) in enumerate(zip(images, base_t_tcps)):
        cv2.imwrite(f'sweep/{i}.png', img)
        base_t_tcp.save(f'sweep/base_t_tcp_{i}')


if __name__ == '__main__':
    main()
