from rl_env import Sim
import numpy as np
import pybullet as p
from tqdm import tqdm
import random
from util import render_camera, next_path
import os
from PIL import Image
import pybullet_data
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

save_error_path = './gerror/'
os.makedirs(save_error_path, mode=0o777, exist_ok=True)


def task(index):
    error_gfile_path = []
    for j in range(120000):
        env = Sim(gui=False, discrete=True)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        env.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
        env.spawnobjects()
        state_save_path = '/media/rl/data/gstate/'+str(int(index))+'/'+str(j)+'.bullet'
        p.restoreState(fileName=state_save_path)
        process_depth, sim_float, mask = render_camera()
        for object_index_in_mask in range(1, 8):
            total_good_points = len(np.where(mask == object_index_in_mask)[0])
            if total_good_points == 0:
                error_gfile_path.append(state_save_path)
                break
        p.disconnect()
    save_path = save_error_path + str(int(index)) + '.npy'
    np.save(save_path, error_gfile_path)



if __name__ == '__main__':
    with ProcessPoolExecutor(max_workers=60) as executor:
        for _ in executor.map(task, range(300)):
            pass
