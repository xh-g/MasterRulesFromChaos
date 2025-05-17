import os
from glob import glob
from tqdm import tqdm


path = '/media/rl/data/state'
folder_names = os.listdir(path)
for folder_name in folder_names:
    os.rename(path + folder_name, path + 'a'+folder_name)

folder_names = os.listdir(path)
number_start = 0
for index, folder_name in enumerate(folder_names):
    os.rename(path + folder_name, path + str(index+number_start))

# path = '/media/rl/data/rstate'
# folder_names = os.listdir(path)
# for folder_name in tqdm(folder_names):
#     target_path = os.path.join(path, folder_name)
#     filenames = os.listdir(target_path)
#     for index, filename in tqdm(enumerate(filenames), leave=False):
#         os.rename(os.path.join(target_path, filename), os.path.join(target_path, str(index)+'.bullet'))
