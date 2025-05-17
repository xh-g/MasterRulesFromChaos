import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import shutil
import os
per_list = []
a = 0
ac= 0
b = 0
bc= 0
c = 0
cc= 0
for i in range(len(glob.glob('./obj_poses/img-*.png'))):
    img_path = './obj_poses/img-' + str(i+1) + '.png'
    npy_path = './obj_poses/pose-' + str(i+1) + '.npy'
    # Read the input image
    img = cv2.imread(img_path)

    # convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray[np.where(gray == 255)] = 0
    gray[np.where(gray != 0)] = 255
    plt.imshow(gray)

    # Apply thresholding in the gray image to create a binary image
    ret, thresh = cv2.threshold(gray, 150, 255, 0)
    plt.imshow(thresh)
    # Find the contours using binary image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print("Number of contours in image:", len(contours))
    
    perimeter = 0
    area = 0
    for j in range(len(contours)):
        cnt = contours[j]
        area += cv2.contourArea(cnt)
        perimeter += cv2.arcLength(cnt, True)
        perimeter = round(perimeter, 4)
    print('Area:', area)
    print('Perimeter:', perimeter)
    per_list.append(perimeter)

    if perimeter > 420:
        dst = './obj_pose_easy/'
        os.makedirs(dst, mode=0o777, exist_ok=True)
        a +=perimeter
        ac +=1
    elif 420 > perimeter > 350:
        dst = './obj_pose_middle/'
        os.makedirs(dst, mode=0o777, exist_ok=True)
        b +=perimeter
        bc +=1
    elif 350 > perimeter:
        dst = './obj_pose_hard/'
        os.makedirs(dst, mode=0o777, exist_ok=True)
        c +=perimeter
        cc +=1
    else:
        print('wrong')
        break
    shutil.copy(img_path, dst)
    shutil.copy(npy_path, dst)
print(a/ac)
print(b/bc)
print(c/cc)
#%%
n, bins, patches = plt.hist(x=per_list, bins=3, color='#0504aa',
                            alpha=0.7, rwidth=0.85)
plt.grid(axis='y', alpha=0.75)
plt.xlabel('Value')
plt.ylabel('Frequency')  
#%%
# import cv2
# import numpy as np
# import glob
# import matplotlib.pyplot as plt


# img_path = './obj_poses/img-7.png'
# # Read the input image
# img = cv2.imread(img_path)

# # convert the image to grayscale
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# gray[np.where(gray == 255)] = 0
# gray[np.where(gray != 0)] = 255
# plt.imshow(gray)

# # Apply thresholding in the gray image to create a binary image
# ret, thresh = cv2.threshold(gray, 150, 255, 0)
# plt.imshow(thresh)
# # Find the contours using binary image
# contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# print("Number of contours in image:", len(contours))

# perimeter = 0
# area = 0
# for j in range(len(contours)):
#     cnt = contours[j]
#     area += cv2.contourArea(cnt)
#     perimeter += cv2.arcLength(cnt, True)
#     perimeter = round(perimeter, 4)
# print('Area:', area)
# print('Perimeter:', perimeter)
