import torch
from bc_common import Net
from rl_env import Sim
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
net = Net()
net.load_state_dict(torch.load('bc_model_best.pth', map_location=torch.device('cpu')))

ft = 0
tt = 0
for i in range(1, 16):
    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
    env.use_old_data = True
    env.test_random = False
    env.use_oracle = True
    env.pre_load = False
    env.mp_save_path = './oracle_state_easy/'
    env.bc = True
    env.test_pattern_id = i
    obs = env.reset()
    fine_r = 0
    for _ in range(7):
        image_target = obs[0, :, :].reshape(1, 120, 120)
        plt.imshow(obs[0, :, :])

        image_current = obs[1, :, :].reshape(1, 120, 120)
        plt.imshow(obs[1, :, :])

        image_empty = np.zeros((1, 120, 120)).astype(np.uint8)
        image = np.concatenate((image_target, image_current, image_empty), axis=0)

        image = torch.tensor([image / 255]).to(torch.float32)
        action = net(image).flatten().tolist()
        obs, reward, done, _ = env.step(action)
        fine_r += reward
        plt.imshow(obs[1, :, :])

    fine_r = fine_r/7
    ft += fine_r
    coarse_r = len(np.where(np.int32(env.current_image != 255) & (np.int32(env.current_image)
                   == env.target_image))[0])/len(np.where(env.target_image != 255)[0])
    tt += coarse_r
    p.disconnect()

print(ft/15)
print(tt/15)


#%%
ft = 0
tt = 0
for i in range(1, 50):
    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
    env.use_old_data = True
    env.test_random = False
    env.use_oracle = True
    env.pre_load = False
    env.mp_save_path = './oracle_state_middle/'
    env.bc = True
    env.test_pattern_id = i
    obs = env.reset()
    fine_r = 0
    for _ in range(7):
        image_target = obs[0, :, :].reshape(1, 120, 120)
        plt.imshow(obs[0, :, :])

        image_current = obs[1, :, :].reshape(1, 120, 120)
        plt.imshow(obs[1, :, :])

        image_empty = np.zeros((1, 120, 120)).astype(np.uint8)
        image = np.concatenate((image_target, image_current, image_empty), axis=0)

        image = torch.tensor([image / 255]).to(torch.float32)
        action = net(image).flatten().tolist()
        obs, reward, done, _ = env.step(action)
        fine_r += reward
        plt.imshow(obs[1, :, :])

    fine_r = fine_r/7
    ft += fine_r
    coarse_r = len(np.where(np.int32(env.current_image != 255) & (np.int32(env.current_image)
                   == env.target_image))[0])/len(np.where(env.target_image != 255)[0])
    tt += coarse_r
    p.disconnect()

print(ft/49)
print(tt/49)
#%%
ft = 0
tt = 0
for i in range(1, 41):
    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
    env.use_old_data = True
    env.test_random = False
    env.use_oracle = True
    env.pre_load = False
    env.mp_save_path = './oracle_state_hard/'
    env.bc = True
    env.test_pattern_id = i
    obs = env.reset()
    fine_r = 0
    for _ in range(7):
        image_target = obs[0, :, :].reshape(1, 120, 120)
        plt.imshow(obs[0, :, :])

        image_current = obs[1, :, :].reshape(1, 120, 120)
        plt.imshow(obs[1, :, :])

        image_empty = np.zeros((1, 120, 120)).astype(np.uint8)
        image = np.concatenate((image_target, image_current, image_empty), axis=0)

        image = torch.tensor([image / 255]).to(torch.float32)
        action = net(image).flatten().tolist()
        obs, reward, done, _ = env.step(action)
        fine_r += reward
        plt.imshow(obs[1, :, :])

    fine_r = fine_r/7
    ft += fine_r
    coarse_r = len(np.where(np.int32(env.current_image != 255) & (np.int32(env.current_image)
                   == env.target_image))[0])/len(np.where(env.target_image != 255)[0])
    tt += coarse_r
    p.disconnect()

print(ft/40)
print(tt/40)
