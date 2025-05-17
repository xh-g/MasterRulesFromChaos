from rl_env import Sim
import numpy as np
import pybullet as p
from tqdm import tqdm
import random
from util import render_camera, next_path
import os
from PIL import Image
import pybullet_data
# env = Sim(gui=False, discrete=True)
# for i in tqdm(range(300)):
#     for j in range(120000):
#         p.resetSimulation()
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         env.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
#         env.spawnobjects()
#         state_save_path = '/media/rl/data/gstate/'+str(i)+'/'+str(j)+'.bullet'
#         p.restoreState(fileName=state_save_path)

# %%
save_img_path = './gimg/'
os.makedirs(save_img_path, mode=0o777, exist_ok=True)
env = Sim(gui=False, discrete=True)
for _ in range(100):
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    env.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
    env.spawnobjects()
    state_save_path = '/media/rl/data/gstate/'+str(random.randint(0, 300))+'/'+str(random.randint(0, 120000))+'.bullet'
    p.restoreState(fileName=state_save_path)
    process_depth, sim_float, mask = render_camera()
    image_save_path = next_path(save_img_path + '%s.png')
    Image.fromarray(np.uint8(process_depth), mode='L').save(image_save_path)
