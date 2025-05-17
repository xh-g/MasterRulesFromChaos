from ur_l515 import L515
from ur_robot import RealRobot
import math
import matplotlib.pyplot as plt
import pybullet as p
from rl_env import Sim
from util import render_camera
import numpy as np
import stable_baselines3
import os
import serial
import serial.tools.list_ports
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

def get_l515_image():
    verts, color_image = L515.get_verts()
    cz = verts[:, :, 2]
    crop_cz = cz[64-60:, 80-60:80+60]
    fill_zero = np.ones(4*120).reshape(4, 120)
    crop_cz = np.vstack([crop_cz, fill_zero])
    crop_cz[np.where(crop_cz > 0.251)] = 255
    crop_cz[np.where(crop_cz != 255)] = 128
    crop_cz = np.rot90(crop_cz)
    crop_cz = np.rot90(crop_cz)
    return crop_cz

L515 = L515()
l515_image = get_l515_image()
plt.imshow(l515_image)
L515.stop_streaming()
