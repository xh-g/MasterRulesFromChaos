import matplotlib.pyplot as plt
import pybullet as p
from rl_env import Sim
from util import render_camera
import numpy as np
# %%
cz = np.load('cz.npy')
plt.imshow(cz)
# crop_cz = cz[:,320-240:320+240]
crop_cz = cz[257-240:, 320-240:320+240]
fill_zero = np.zeros(17*480).reshape(17, 480)
crop_cz = np.vstack([crop_cz, fill_zero])
plt.imshow(crop_cz)
crop_cz[np.where(crop_cz > 0.245)] = 0
plt.imshow(crop_cz)
crop_cz[:120, :] = 0
crop_cz[:,:100] = 0
crop_cz[400:, :] = 0
plt.imshow(crop_cz)
crop_cz[np.where(crop_cz == 0)] = 255
crop_cz[np.where(crop_cz < 0.245)] = 128
plt.imshow(crop_cz)
# %%
env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
p.loadURDF('./model/10x10x1.urdf', [0, 0, 0.02/2], useFixedBase=True)
# _, sim_float, _ = render_camera(camera_position=[-0.002, 0.002, 0.25],
#                                 camera_target=[-0.002, 0.002, 0],
#                                 camera_fov=55.371784673413806,
#                                 high_res=True)
_, sim_float, _ = render_camera(camera_position=[0, 0, 0.25],
                                camera_target=[0, 0, 0],
                                camera_fov=55.371784673413806,
                                high_res=True)
sim_float[np.where(sim_float > 0.245)] = 255
sim_float[np.where(sim_float < 0.245)] = 128
plt.imshow(sim_float)

print(len(np.where(crop_cz == 128)[0]))
print(len(np.where(sim_float == 128)[0]))
#%%
diff_z = crop_cz - sim_float
plt.imshow(diff_z)
