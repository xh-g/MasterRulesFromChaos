from rl_env import Sim
from PIL import Image
from util import render_camera, next_path
import matplotlib.pyplot as plt
import pybullet as p
import numpy as np
import os
import math
# save_path = './obj_poses/'
# os.makedirs(save_path, mode=0o777, exist_ok=True)
env = Sim(gui=False, discrete=True)

# %%
# obj_poses = np.load('pose-1.npy')
ob1_pos = [0, 0, 0]
# ob2_pos = [obj_poses[1][0], obj_poses[1][1], 0]
# ob3_pos = [obj_poses[2][0], obj_poses[2][1], 0]
# ob4_pos = [obj_poses[3][0], obj_poses[3][1], 0]
# ob5_pos = [obj_poses[4][0], obj_poses[4][1], 0]
# ob6_pos = [obj_poses[5][0], obj_poses[5][1], 0]
# ob7_pos = [obj_poses[6][0], obj_poses[6][1], 0]

or1 = p.getQuaternionFromEuler([0,0,math.radians(45)])
# or2 = p.getQuaternionFromEuler([0,0,obj_poses[1][2]])
# or3 = p.getQuaternionFromEuler([0,0,obj_poses[2][2]])
# or4 = p.getQuaternionFromEuler([0,0,obj_poses[3][2]])
# or5 = p.getQuaternionFromEuler([0,0,obj_poses[4][2]])
# or6 = p.getQuaternionFromEuler([0,0,obj_poses[5][2]])
# or7 = p.getQuaternionFromEuler([0,0,obj_poses[6][2]])
# %%
ob1_id = p.loadURDF('./model/1.urdf', ob1_pos, or1)
# # %%
# ob2_id = p.loadURDF('./model/2.urdf', ob2_pos, or2)
# # %%
# ob3_id = p.loadURDF('./model/3.urdf', ob3_pos, or3)
# # %%
# ob4_id = p.loadURDF('./model/4.urdf', ob4_pos, or4)
# # %%
# ob5_id = p.loadURDF('./model/5.urdf', ob5_pos, or5)
# # %%
# ob6_id = p.loadURDF('./model/6.urdf', ob6_pos, or6)
# # %%
# ob7_id = p.loadURDF('./model/7.urdf', ob7_pos, or7)
# %% show
process_depth, sim_float, mask = render_camera()
plt.imshow(process_depth)
#%%
p.disconnect()

