from ur_l515 import L515
from ur_robot import RealRobot
import math
import matplotlib.pyplot as plt
import pybullet as p
from rl_env import Sim
from util import render_camera, next_path
import numpy as np
import stable_baselines3
import os
import serial
import serial.tools.list_ports
from PIL import Image
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
ports = list(serial.tools.list_ports.comports())
for port in ports:
    print(port)
os.system("echo 1212 | sudo -S chmod 666 /dev/ttyACM0")


def get_l515_image():
    verts, color_image = L515.get_verts()
    cz = verts[:, :, 2]
    crop_cz = cz[64-60:, 80-60:80+60]
    fill_zero = np.ones(4*120).reshape(4, 120)
    crop_cz = np.vstack([crop_cz, fill_zero])
    crop_cz[np.where(crop_cz > 0.25)] = 255
    crop_cz[np.where(crop_cz != 255)] = 128
    crop_cz = np.rot90(crop_cz)
    crop_cz = np.rot90(crop_cz)
    color_image = np.rot90(color_image)
    color_image = np.rot90(color_image)
    return crop_cz, color_image


env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=True)
model = stable_baselines3.PPO.load('./4-1.zip', env=env)
print('model load')
rob = RealRobot(has_gripper=True)
rob.set_tcp(0, 0, 0, 0, 0, 0)
# %%
L515 = L515()
rob.go_c_home(0.1, 0.1)
env.test_pattern_id = 5
env.mp_save_path = './fg7_failure_cases/'
obs = env.reset()
plt.imshow(obs[0, :, :])
# %%
save_path = './video/'
save_path = next_path(save_path + '%s/')
os.makedirs(save_path, mode=0o777, exist_ok=True)
#%%
for step in range(1, 8):
    rob.apply_action_on_real_robot(step=step,o_x=0,o_y=0,o_yaw=0,pick=True)
    if step >1:
        l515_image, _ = get_l515_image()
        obs = np.stack((env.target_image, l515_image), axis=0)
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _ = env.step(action)
    
    rob.apply_action_on_real_robot(step=step,
                                   o_x=((action[1] * 2 - 50) * 0.002*-1),
                                   o_y=((action[0] * 2 - 50) * 0.002),
                                   o_yaw=action[2] * 45 - 180,
                                   pick=False)
    if step >1:
        Image.fromarray(np.uint8(l515_image), mode="L").save(next_path(save_path + '%s_p.png'))
    if done is True:
        break
# %%
L515.stop_streaming()
