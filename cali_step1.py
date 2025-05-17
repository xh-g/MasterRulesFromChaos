from ur_l515 import L515
from ur_robot import RealRobot
import math
import matplotlib.pyplot as plt
import pybullet as p
from rl_env import Sim
from util import render_camera
import numpy as np
rob = RealRobot(has_gripper=False)
rob.set_tcp(0, 0, 0, 0, 0, 0)
rob.go_c_home(0.05, 0.05)
# rob.set_gripper_ori(roll=0, pitch=math.radians(3), yaw=0, acc=0.1, vel=0.1, wait=True)
#%%
L515 = L515()
verts, color_image = L515.get_full_dpi_verts()
L515.stop_streaming()
# %%
cz = verts[:, :, 2]
plt.imshow(cz)
np.save('cz.npy',cz)
# %%
crop_cz = cz[257-240:,320-240:320+240]
fill_zero = np.zeros(17*480).reshape(17,480)
crop_cz =np.vstack([crop_cz, fill_zero])
plt.imshow(crop_cz)
#%%
crop_cz[np.where(crop_cz>0.245)] = 0
plt.imshow(crop_cz)
# %%
# cz[np.where(cz > 0.41)] = 1
# cp_w = 82
# cz = cz[:, cp_w - 60: cp_w + 60]
# plt.imshow(cz)
# %%
# plt.imshow(color_image)
# 
# %%

# width is:  640
# height is:  480
# ppx is:  319.73828125 / 159.869140625
# ppy is:  256.953125 /128.4765625
# fx is:  457.3828125
# fy is:  457.4296875
# HFOV is 69.95303419754133
# VFOV is 55.371784673413806
