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

save_path = './test_mp/'
model_path = './4-1.zip'
e_ft = []
e_tt = []
scale = 1

def save_pose_to_bullet(obj_poses):
    os.makedirs(save_path, mode=0o777, exist_ok=True)
    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
    env.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
    ob1_pos = [obj_poses[0][0]*scale, obj_poses[0][1]*scale, 0]
    ob2_pos = [obj_poses[1][0]*scale, obj_poses[1][1]*scale, 0]
    ob3_pos = [obj_poses[2][0]*scale, obj_poses[2][1]*scale, 0]
    ob4_pos = [obj_poses[3][0]*scale, obj_poses[3][1]*scale, 0]
    ob5_pos = [obj_poses[4][0]*scale, obj_poses[4][1]*scale, 0]
    ob6_pos = [obj_poses[5][0]*scale, obj_poses[5][1]*scale, 0]
    ob7_pos = [obj_poses[6][0]*scale, obj_poses[6][1]*scale, 0]
    or1 = p.getQuaternionFromEuler([0, 0, obj_poses[0][2]])
    or2 = p.getQuaternionFromEuler([0, 0, obj_poses[1][2]])
    or3 = p.getQuaternionFromEuler([0, 0, obj_poses[2][2]])
    or4 = p.getQuaternionFromEuler([0, 0, obj_poses[3][2]])
    or5 = p.getQuaternionFromEuler([0, 0, obj_poses[4][2]])
    or6 = p.getQuaternionFromEuler([0, 0, obj_poses[5][2]])
    or7 = p.getQuaternionFromEuler([0, 0, obj_poses[6][2]])
    ob1_id = p.loadURDF('./model/1.urdf', ob1_pos, or1)
    ob2_id = p.loadURDF('./model/2.urdf', ob2_pos, or2)
    ob3_id = p.loadURDF('./model/3.urdf', ob3_pos, or3)
    ob4_id = p.loadURDF('./model/4.urdf', ob4_pos, or4)
    ob5_id = p.loadURDF('./model/5.urdf', ob5_pos, or5)
    ob6_id = p.loadURDF('./model/6.urdf', ob6_pos, or6)
    ob7_id = p.loadURDF('./model/7.urdf', ob7_pos, or7)
    process_depth, sim_float, mask = render_camera()
    p.saveBullet(next_path(save_path + '%s.bullet'))
    Image.fromarray(np.uint8(process_depth), mode="L").save(next_path(save_path + '%s.png'))
    p.disconnect()


def predict():
    a = []
    b = []
    ft = 0
    tt = 0
    for i in range(len(glob.glob(save_path+'*.bullet'))):
        env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
        model = stable_baselines3.PPO.load(model_path, env=env)
        env.test_pattern_id = i + 1
        obs = env.reset()

        fine_r = 0
        for _ in range(7):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            fine_r += reward
            if done is True:
                break
        fine_r = fine_r/7
        ft += fine_r
        a.append(fine_r)
        b.append(env.test_pattern_id)
        coarse_r = len(np.where(np.int32(env.current_image != 255) & (np.int32(env.current_image)
                       == env.target_image))[0])/len(np.where(env.target_image != 255)[0])
        tt += coarse_r
        Image.fromarray(np.uint8(env.current_image), mode="L").save(next_path(save_path + '%s_p.png'))
        # np.save(next_path(save_path + '%s_r.npy'), [fine_r, coarse_r])
        p.disconnect()
    e_ft.append(ft/len(glob.glob(save_path+'*.bullet')))
    e_tt.append(tt/len(glob.glob(save_path+'*.bullet')))
    # shutil.rmtree(save_path)

#%%
# save pose to bullet state
for file_name in glob.glob('./obj_pose_easy/*.npy'):
    obj_poses = np.load(file_name)
    save_pose_to_bullet(obj_poses)
predict()
shutil.rmtree(save_path)
#%%
for file_name in glob.glob('./obj_pose_middle/*.npy'):
    obj_poses = np.load(file_name)
    save_pose_to_bullet(obj_poses)
predict()
shutil.rmtree(save_path)
#%%
for file_name in glob.glob('./obj_pose_hard/*.npy'):
    obj_poses = np.load(file_name)
    save_pose_to_bullet(obj_poses)
predict()
shutil.rmtree(save_path)
#%%
print(list(np.around(np.array(e_ft), 3)))
print(list(np.around(np.array(e_tt), 3)))
