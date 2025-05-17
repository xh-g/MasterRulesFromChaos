from rl_env import Sim
from util import render_camera
import matplotlib.pyplot as plt
import pybullet as p
import numpy as np
import pybullet_data
env = Sim(gui=False, discrete=True)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
env.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
# env.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
env.spawnobjects()
state_save_path = './14863.bullet'
p.restoreState(fileName=state_save_path)
process_depth, sim_float, mask = render_camera()
plt.imshow(process_depth)
