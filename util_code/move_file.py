import shutil
import numpy as np
import os
error_list = []
for i in range(300):
    file = np.load('./gerror/' + str(i) + '.npy').tolist()
    if len(file) == 0:
        pass
    else:
        for j in range(len(file)):
            error_list.append(file[j])
for index, file_name in enumerate(error_list):
    rp_path = './rpst/'+str(index)+'.bullet'
    # print(file_name, index)
    # os.remove(file_name)
    shutil.copy(rp_path, file_name) 
