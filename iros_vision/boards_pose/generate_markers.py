import numpy as np
import cv2
import cv2.aruco as aruco
from PIL import Image

"""
Generates the marker images and prints the size the images should have when printed
"""

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)

m_per_inch = 0.0254

# second parameter is id number
# last parameter is total image size
size_px = 501
pad_px = size_px // 7
tot_size_px = size_px + 2 * pad_px
line_px = pad_px * 5
size_m = 0.02
dpi = size_px / size_m * m_per_inch
s = pad_px * 2 + size_px + line_px
m = pad_px + size_px // 2

c = 0, 0, 0
lw = 3

print('width [cm]:', s / dpi * m_per_inch * 100)

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1.8
text_thickness = 2

square_names = 'TASK', 'KIT'
corners = 'TL', 'TR', 'BL', 'BR'

i = 0
for name in square_names:
    for corner in corners:
        text = f'{name}:{corner}'

        tw, th = cv2.getTextSize(text, font, font_scale, text_thickness)[0]
        assert tw <= size_px

        img = np.ones((s, s, 3), dtype=np.uint8) * 255
        cv2.rectangle(img, (0, s), (tot_size_px, s - tot_size_px), c, lw)
        cv2.rectangle(img, (m, line_px), (m + pad_px, 0), c, lw)
        cv2.rectangle(img, (tot_size_px, s - m), (s, s - m - pad_px), c, lw)

        img[-size_px - pad_px:-pad_px, pad_px:pad_px + size_px] = aruco.drawMarker(aruco_dict, i, size_px)[..., None]
        cv2.putText(img, text, (tot_size_px + pad_px // 2, s - m - round(th * 0.4)),
                    font, font_scale, 0, text_thickness)

        img = Image.fromarray(img)
        img.save(f'markers/{text}.png', dpi=(dpi, dpi))

        i += 1
