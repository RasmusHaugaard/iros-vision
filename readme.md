# iros-vision

### install

```
$ pip3 install git+git://github.com/RasmusHaugaard/ur-control.git

$ git clone httpsgit://github.com/RasmusHaugaard/iros-vision.git
$ cd iros-vision
$ pip3 install -e .
```


### usage
Make sure the camera A node is running on the vision computer:
```
$ cd iros-vision/scripts
$ roslaunch camera_A_node.launch
```
And that the images are rectified:
```
$ cd iros-vision/scripts
$ python3 rectify.py
```

Example getting the board poses:
```python
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
```
and using them:
```python
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
```

You can also load camera calibrations and the servo table_t_base:
```python
import iros_vision
table_t_base = iros_vision.load_table_t_base('servo')
tcp_t_cam = iros_vision.load_tcp_t_cam('servo/white')  # servo/black, A
K_white, dist_coeffs_white, h, w = iros_vision.load_cam_intrinsics('servo/white') # servo/black, A
```
All poses are given as transform3d.Transform