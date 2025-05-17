import shutil
from rl_env import Sim
from PIL import Image
from util import render_camera, next_path
import matplotlib.pyplot as plt
import pybullet as p
import numpy as np
import os
import stable_baselines3
import glob
from tqdm import tqdm
save_path = './test_random/'
os.makedirs(save_path, mode=0o777, exist_ok=True)
model_path = './5-1.zip'
test_number = 10
# predirct action and save performance
ft = 0
fc = 0
for i in tqdm(range(test_number)):
    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
    model = stable_baselines3.PPO.load(model_path, env=env)
    env.test_random = True
    obs = env.reset()

    fine_r = 0
    for _ in range(7):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, _ = env.step(action)
        fine_r += reward
        if done is True:
            break
    fine_r = fine_r/7
    coarse_r = len(np.where(np.int32(env.current_image != 255) & (np.int32(env.current_image)
                   == env.target_image))[0])/len(np.where(env.target_image != 255)[0])
    ft += fine_r
    fc += coarse_r
    Image.fromarray(np.uint8(env.target_image), mode="L").save(next_path(save_path + '%s_t.png'))
    Image.fromarray(np.uint8(env.current_image), mode="L").save(next_path(save_path + '%s_p.png'))
    # np.save(next_path(save_path + '%s_r.npy'), [fine_r, coarse_r])
    p.disconnect()
print('fine_r:', ft/test_number)
print('coarse_r:', fc/test_number)
# %%
shutil.rmtree(save_path)
