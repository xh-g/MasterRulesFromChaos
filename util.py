import numpy as np
import random
import pybullet as p
import os


def render_camera(camera_position=[0, 0, 0.25], camera_target=[0, 0, 0], camera_fov=55.371784673413806, for_target_render=None, high_res=False):

    if high_res is False:
        IMAGE_WIDTH = 160
        IMAGE_HEIGHT = 120
    else:
        IMAGE_WIDTH = 640
        IMAGE_HEIGHT = 480

    cut_pix = int((IMAGE_WIDTH-IMAGE_HEIGHT)/2)

    CAMERA_FAR = 0.3
    CAMERA_NEAR = 0.15
    HFOV_VFOV = 320/240
    if for_target_render is None:
        UpVector = [0, 1, 0]
    else:
        UpVector = for_target_render

    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_position,
                                      cameraTargetPosition=camera_target,
                                      cameraUpVector=UpVector)
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=camera_fov, aspect=HFOV_VFOV, nearVal=CAMERA_NEAR, farVal=CAMERA_FAR)
    _, _, _, depth_image, mask = p.getCameraImage(
        width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    # Depth image
    depth_image = np.array(depth_image).reshape(IMAGE_HEIGHT, IMAGE_WIDTH)
    mask = np.array(mask).reshape(IMAGE_HEIGHT, IMAGE_WIDTH)

    depth_image = CAMERA_FAR * CAMERA_NEAR / \
        (CAMERA_FAR - (CAMERA_FAR - CAMERA_NEAR) * depth_image)
    depth_image = np.array(depth_image).reshape(IMAGE_HEIGHT, IMAGE_WIDTH)

    depth_image = depth_image[:, cut_pix:IMAGE_WIDTH-cut_pix]
    mask = mask[:, cut_pix:IMAGE_WIDTH-cut_pix]

    sim_float = depth_image.copy()

    process_depth = depth_image.copy()
    process_depth[np.where(sim_float > 0.245)] = 255
    process_depth[np.where(sim_float < 0.245)] = 128

    # min_in_d = 0.15
    # max_in_d = 0.3
    # process_depth = depth_image.copy()
    # process_depth = (process_depth-min_in_d)/(max_in_d-min_in_d)*255
    # process_depth = np.array(process_depth).astype(np.uint8)

    return process_depth, sim_float, mask


def discretize(value, possibilities):
    closest_value = possibilities[0]
    for i in range(len(possibilities)):
        if abs(value - possibilities[i]) < abs(value - closest_value):
            closest_value = possibilities[i]
    return closest_value


def rescale(x, x_min, x_max, y_min, y_max):
    return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min


def normalize_minus_11(x, old_range):
    return rescale(x, old_range[0], old_range[1], -1, 1)


def unnormalize_minus_11(x, new_range):
    return rescale(x, -1, 1, new_range[0], new_range[1])


def normalize_01(x, old_range):
    return rescale(x, old_range[0], old_range[1], 0, 1)


def unnormalize_01(x, new_range):
    return rescale(x, 0, 1, new_range[0], new_range[1])


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def next_path(path_pattern):
    """
    Finds the next free path in an sequentially named list of files
    e.g. path_pattern = 'file-%s.txt':
    file-1.txt
    file-2.txt
    file-3.txt
    Runs in log(n) time where n is the number of existing files in sequence
    """
    i = 1

    # First do an exponential search
    while os.path.exists(path_pattern % i):
        i = i * 2

    # Result lies somewhere in the interval (i/2..i]
    # We call this interval (a..b] and narrow it down until a + 1 = b
    a, b = (i // 2, i)
    while a + 1 < b:
        c = (a + b) // 2  # interval midpoint
        a, b = (c, b) if os.path.exists(path_pattern % c) else (a, c)

    return path_pattern % b


gripper_limits = {'steps': [0, 8]}

objects_limits = {'o_x': [-0.07, 0.07],
                  'o_y': [-0.07, 0.07],
                  'o_yaw': [-1.57, 1.57]}
