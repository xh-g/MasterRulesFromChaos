import numpy as np
import pybullet as p
from rl_env import Sim
from torchvision.io import write_jpeg
import torch
import argparse
import os
from tqdm import tqdm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', type=str, default='train')
    parser.add_argument('--size', type=int, default=50000)
    args = parser.parse_args()

    env = Sim(gui=False, discrete=True, cnn_policy=True, play_only=False)
    dataset_type = args.type

    # Make dataset directory if it doesn't exist
    if not os.path.exists(f'dataset/{dataset_type}'):
        os.makedirs(f'dataset/{dataset_type}')

    episode = 0
    for i in tqdm(range(args.size)):
        if episode > 1000:
            episode = 0
            p.disconnect()
            p.connect(p.DIRECT)
        else:
            p.resetSimulation()
            episode +=1

        env.play_only = False
        env.use_old_data = True
        env.test_pattern_id = None
        env.test_random = False
        env.use_oracle = True
        env.pre_load = True
        env.bc = True
        env.reset()

        # Input
        image_target = env.target_image.reshape(1, 120, 120).astype(np.uint8)
        image_current = env.current_image.reshape(1, 120, 120).astype(np.uint8)
        image_empty = np.zeros((1, 120, 120)).astype(np.uint8)
        image = np.concatenate((image_target, image_current, image_empty), axis=0)

        # Output
        position_3d = env.target_pos[env.load_x]
        orientation_3d = env.target_ori[env.load_x]
        _, _, target_yaw = p.getEulerFromQuaternion(orientation_3d)
        label = np.array([[position_3d[0], position_3d[1], target_yaw]], dtype=np.float32)

        # Store
        write_jpeg(torch.from_numpy(image), f'dataset/{dataset_type}/{i}.jpg')
        np.savetxt(f'dataset/{dataset_type}/{i}.csv', label, delimiter=',')


if __name__ == '__main__':
    main()
