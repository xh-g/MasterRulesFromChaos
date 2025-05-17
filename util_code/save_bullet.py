from rl_env import Sim
import pybullet as p
import os
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import numpy as np
import time
from itertools import repeat
from tqdm import tqdm
# from PIL import Image

# print(multiprocessing.cpu_count())


def task(save_state_path, index):
    env = Sim(gui=False, discrete=True)
    collect_number = int(1000)
    for i in range(collect_number):
        # dictionary = env.build_and_save_environment()
        env.build_and_save_environment()
        state_save_path = save_state_path + str(index)+'_' + str(i)+'.bullet'
        p.saveBullet(state_save_path)

    p.disconnect()


def main(save_state_path):
    with ProcessPoolExecutor(max_workers=multiprocessing.cpu_count()-8) as executor:
        for _ in executor.map(task, repeat(save_state_path), range(120)):
            pass


if __name__ == '__main__':
    # 0.12 million * 200
    collect_number = 100

    for f_id in tqdm(range(collect_number)):
        localtime = time.asctime(time.localtime(time.time()))
        print("start:", localtime)
        path = '/media/administrator/data/rstate/' + str(int(f_id)) + '/'
        os.makedirs(path, mode=0o777, exist_ok=True)
        main(path)
        localtime = time.asctime(time.localtime(time.time()))
        print("end:", localtime)

    # localtime = time.asctime(time.localtime(time.time()))
    # print("start:", localtime)
    # main(save_state_path='/media/administrator/data/state/')
    # main(save_state_path='./state/')
    # localtime = time.asctime(time.localtime(time.time()))
    # print("end:", localtime)

# import os
# # Function to rename multiple files
# def main():
# 	i = 0
# 	path="E:/amit/"
# 	for filename in os.listdir(path):
# 		my_dest ="new" + str(i) + ".jpg"
# 		my_source =path + filename
# 		my_dest =path + my_dest
# 		# rename() function will
# 		# rename all the files
# 		os.rename(my_source, my_dest)
# 		i += 1
# # Driver Code
# if __name__ == '__main__':
# 	# Calling main() function
# 	main()
