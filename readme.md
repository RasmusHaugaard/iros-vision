# iros-vision

### install

```
$ pip3 install git+git://github.com/RasmusHaugaard/ur-control.git
```
```
$ pip3 install git+git://github.com/RasmusHaugaard/iros-vision.git
```


### usage

```python
import iros_vision
from ur_control import Robot

r = Robot.from_ip('192.168.1.130')
# or: r = Robot(rtde_r, rtde_c)

# The below function moves robot A above the workspace. 
# the other robots must be clear of the workspace
base_t_taskboard, base_t_kit = iros_vision.capture_images_and_find_boards(r)
# all poses are given as transform3d.Transform

# you can also load camera calibrations and the servo table_t_base
table_t_base = iros_vision.load_table_t_base('servo')
tcp_t_cam = iros_vision.load_tcp_t_cam('servo/white')  # servo/black, A
K_white, dist_coeffs_white = iros_vision.load_cam_intrinsics('servo/white') # servo/black, A
```

