import os
import numpy as np
from PIL import Image

# 이미지 크기와 sin 함수의 주기, 진폭, 위상 등을 설정합니다.
width = 1000
height = 1000
period = 100
amplitude = 0.1
phase = 0

# sin 함수를 이용하여 이미지 데이터를 생성합니다.
y, x = np.indices((height, width))
data = amplitude * np.sin(2 * np.pi * (1.0 / period) * x + phase)

# 생성한 이미지 데이터를 이미지 파일로 저장합니다.
img = Image.fromarray(np.uint8(data * 255))

# 이미지 파일을 저장할 경로를 설정합니다.
path = os.path.expanduser('/home/rtlink/jiwon/socap_ws/src/gazebo_world/worlds/materials/textures/sin_wave.png')

# 이미지 파일을 저장합니다.
img.save(path)
