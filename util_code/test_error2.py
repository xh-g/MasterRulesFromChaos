from rl_env import Sim
import numpy as np
import pybullet as p
from tqdm import tqdm
import random
from util import render_camera, next_path
import os
from PIL import Image
import pybullet_data
error_rfile_path = []
env = Sim(gui=False, discrete=True)
for i in tqdm(range(300)):
    for j in tqdm(range(120000), leave=False):
        p.resetSimulation()
        env.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
        env.spawnobjects()
        state_save_path = '/media/rl/data/rstate/'+str(i)+'/'+str(j)+'.bullet'
        p.restoreState(fileName=state_save_path)
        process_depth, sim_float, mask = render_camera()
        for object_index_in_mask in range(1, 8):
            total_good_points = len(np.where(mask == object_index_in_mask)[0])
            if total_good_points == 0:
                print('i:', i, "j:", j)
                error_rfile_path.append(state_save_path)
                break
np.save('error_rfile_path.npy', error_rfile_path)