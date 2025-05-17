import os

target_path = './obj_poses_hard/'
filenames = os.listdir(target_path)
for name in filenames:
    new_name = target_path + name.split('-')[0] + '-' + str(int(name.split('-')[1].split('.')[0])+70) + '.'+name.split('-')[1].split('.')[1]
    
    os.rename(os.path.join(target_path, name), new_name)
