import numpy as np
import gym
import pybullet as p
import random
import math
import time
from util import normalize_01, render_camera, gripper_limits
import pybullet_data


ACTION_SIZE = 3  # x y yaw
VECTOR_SIZE = 6


class Sim(gym.Env):
    def __init__(self, gui=False, discrete=True, cnn_policy=False, play_only=False):

        self.target_image = None
        self.target_mask = None
        self.target_objects = []
        self.target_pos = []
        self.target_ori = []
        self.current_image = None
        self.current_mask = None
        self.current_objects = []
        # rl relative
        self.steps = 0
        self.episode = 0

        self.input_image = None
        self.use_gui = gui
        self.discrete = discrete
        self.cnn_policy = cnn_policy

        # for speed_up
        self.first_current_image = np.ones(120*120).reshape(120, 120)*255

        # for test
        self.play_only = play_only
        self.test_random = False
        self.use_old_data = True
        self.use_oracle = True
        self.pre_load = True
        self.bc = False
        if self.play_only is True:
            self.pre_load = False

        self.mp_save_path = './test_mp/'
        self.test_pattern_id = None

        if self.use_gui:
            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.3,
                cameraYaw=50.8,
                cameraPitch=-44.2,
                cameraTargetPosition=[-0.56, 0.47, -0.52],
            )
        else:
            p.connect(p.DIRECT)

        if self.cnn_policy is True:
            self.observation_space = gym.spaces.Box(low=0, high=255, shape=(2, 120, 120), dtype=np.uint8)
        else:
            self.observation_space = gym.spaces.Dict({
                "vector": gym.spaces.Box(low=-1, high=1, shape=(VECTOR_SIZE,), dtype=np.float32),
                "image": gym.spaces.Box(low=0, high=255, shape=(2, 120, 120), dtype=np.uint8)})

        if self.discrete is True:
            # self.action_space = gym.spaces.MultiDiscrete([101, 101, 9])
            self.action_space = gym.spaces.MultiDiscrete([51, 51, 8])
        else:
            self.action_space = gym.spaces.Box(low=-1, high=1, shape=(ACTION_SIZE,), dtype=np.float32)

    def delay(self, duration):
        for _ in range(duration):
            if self.use_gui is True:
                time.sleep(0.3)
            p.stepSimulation()

    def ge_random_pose(self):
        x_random = random.uniform(-1, 1) * 0.1
        y_random = random.uniform(-1, 1) * 0.1
        # yaw_random = random.uniform(-1, 1) * 1.57
        yaw_random = random.choice([math.radians(-180), math.radians(-135), math.radians(-90), math.radians(-45),
                                   math.radians(0), math.radians(45), math.radians(90), math.radians(135), math.radians(180)])
        pos = [x_random, y_random, 0]
        ori = p.getQuaternionFromEuler([0, 0, yaw_random])
        return pos, ori

    def get_observation(self):
        self.input_image = np.stack((self.target_image, self.current_image), axis=0)

        if self.cnn_policy is True:
            return self.input_image
        else:
            current_step = normalize_01(self.steps, gripper_limits["steps"])
            return {
                "vector": np.array([current_step, current_step, current_step, current_step, current_step, current_step], dtype=np.float32),
                "image": self.input_image,
            }

    def random_camera_yaw(self):
        # return random.choice([[0, 1, 0], [1, 0, 0], [0, -1, 0], [-1, 0, 0]])
        return [0, 1, 0]

    def spawnobjects(self):
        self.selected_object = [1, 2, 3, 4, 5, 6, 7]
        self.target_pos = []
        self.target_ori = []
        self.target_objects = []
        for o_index in self.selected_object:
            pos, ori = self.ge_random_pose()
            uid = p.loadURDF('./model/'+str(o_index)+'.urdf', pos, ori)
            self.target_objects.append(uid)
            self.target_pos.append(pos)
            self.target_ori.append(ori)

    def build_environment(self):

        if self.use_old_data is True:
            # memory leak fix
            if self.episode > 1000:
                self.episode = 0
                p.disconnect()
                p.connect(p.DIRECT)
            else:
                p.resetSimulation()

            if self.play_only is True:
                if self.test_random is True:
                    if random.random() > 0.5:
                        self.drop_ground_objects()
                    else:
                        while True:
                            p.resetSimulation()
                            self.target_objects = []
                            self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                            self.spawnobjects()
                            self.delay(1)
                            if_ok = True
                            for object_id in self.target_objects:
                                if if_ok is False:
                                    break
                                for px in p.getContactPoints(object_id):
                                    if px[2] != self.table_id and px[8] < -0.0005:
                                        if_ok = False
                            if if_ok is True:
                                break
                else:
                    self.target_objects = []
                    self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                    self.spawnobjects()
                    p.restoreState(fileName=self.mp_save_path + str(int(self.test_pattern_id))+'.bullet')

            else:
                if self.use_oracle is True and self.bc is False:
                    if random.random() > 0.9:
                        self.target_objects = []
                        self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                        self.spawnobjects()
                        state_save_path = './oracle_state/'+str(random.randint(1, 104))+'.bullet'
                        p.restoreState(fileName=state_save_path)
                    else:
                        if random.random() > 0.5:
                            self.target_objects = []
                            self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                            self.spawnobjects()
                            state_save_path = '/media/rl/data/rstate/' + \
                                str(random.randint(0, 299))+'/'+str(random.randint(0, 120000-1))+'.bullet'
                            p.restoreState(fileName=state_save_path)
                        else:
                            self.target_objects = []
                            p.setAdditionalSearchPath(pybullet_data.getDataPath())
                            self.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
                            self.spawnobjects()
                            state_save_path = '/media/rl/data/gstate/' + \
                                str(random.randint(0, 299))+'/'+str(random.randint(0, 120000-1))+'.bullet'
                            p.restoreState(fileName=state_save_path)
                if self.use_oracle is True and self.bc is True:
                    self.target_objects = []
                    self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                    self.spawnobjects()
                    state_save_path = './oracle_state/'+str(random.randint(1, 104))+'.bullet'
                    p.restoreState(fileName=state_save_path)
                else:
                    if random.random() > 0.5:
                        self.target_objects = []
                        self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                        self.spawnobjects()
                        state_save_path = '/media/rl/data/rstate/' + \
                            str(random.randint(0, 299))+'/'+str(random.randint(0, 120000-1))+'.bullet'
                        p.restoreState(fileName=state_save_path)
                    else:
                        self.target_objects = []
                        p.setAdditionalSearchPath(pybullet_data.getDataPath())
                        self.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
                        self.spawnobjects()
                        state_save_path = '/media/rl/data/gstate/' + \
                            str(random.randint(0, 299))+'/'+str(random.randint(0, 120000-1))+'.bullet'
                        p.restoreState(fileName=state_save_path)

            self.target_pos = []
            self.target_ori = []
            for o_id in self.target_objects:
                pos, ori = p.getBasePositionAndOrientation(o_id)
                self.target_pos.append(pos)
                self.target_ori.append(ori)

        else:
            if random.random() > 0.5:
                self.drop_ground_objects()
            else:
                while True:
                    p.resetSimulation()
                    self.target_objects = []
                    self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                    self.spawnobjects()
                    self.delay(1)
                    if_ok = True
                    for object_id in self.target_objects:
                        if if_ok is False:
                            break
                        for px in p.getContactPoints(object_id):
                            if px[2] != self.table_id and px[8] < -0.0005:
                                if_ok = False
                    if if_ok is True:
                        break

        self.target_image, _, self.target_mask = render_camera()

        self.random_camera()
        self.a = 0
        if self.pre_load is True:
            self.load_x = random.choice([0, 1, 2, 3, 4, 5, 6])
            if self.load_x == 0:
                for i in range(1, 8):
                    p.removeBody(i)
                self.current_image = self.first_current_image
            elif self.load_x == 1:
                if random.random() > 0.5:
                    p.removeBody(7)
                    p.removeBody(6)
                    p.removeBody(5)
                    p.removeBody(4)
                    p.removeBody(3)
                    p.removeBody(2)
                    self.current_objects.append(1)
                    self.a = 1
                else:
                    p.removeBody(7)
                    p.removeBody(6)
                    p.removeBody(5)
                    p.removeBody(4)
                    p.removeBody(3)
                    p.removeBody(1)
                    self.current_objects.append(2)
                    self.a = 2
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)
            elif self.load_x == 2:
                p.removeBody(7)
                p.removeBody(6)
                p.removeBody(5)
                p.removeBody(4)
                p.removeBody(3)
                self.current_objects.append(1)
                self.current_objects.append(2)
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            elif self.load_x == 3:
                p.removeBody(7)
                p.removeBody(6)
                p.removeBody(5)
                p.removeBody(4)
                self.current_objects.append(1)
                self.current_objects.append(2)
                self.current_objects.append(3)
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            elif self.load_x == 4:
                p.removeBody(7)
                p.removeBody(6)
                p.removeBody(5)
                self.current_objects.append(1)
                self.current_objects.append(2)
                self.current_objects.append(3)
                self.current_objects.append(4)
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            elif self.load_x == 5:
                p.removeBody(7)
                p.removeBody(6)
                self.current_objects.append(1)
                self.current_objects.append(2)
                self.current_objects.append(3)
                self.current_objects.append(4)
                self.current_objects.append(5)
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            elif self.load_x == 6:
                if random.random() > 0.5:
                    p.removeBody(7)
                    self.current_objects.append(1)
                    self.current_objects.append(2)
                    self.current_objects.append(3)
                    self.current_objects.append(4)
                    self.current_objects.append(5)
                    self.current_objects.append(6)
                    self.a = 6
                else:
                    p.removeBody(6)
                    self.current_objects.append(1)
                    self.current_objects.append(2)
                    self.current_objects.append(3)
                    self.current_objects.append(4)
                    self.current_objects.append(5)
                    self.current_objects.append(7)
                    self.a = 7
                p.performCollisionDetection()
                self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)
        else:
            p.resetSimulation()
            self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
            self.current_image = self.first_current_image

    def random_camera(self):
        self.camera_position = [0, 0, 0.25]
        self.camera_target = [0, 0, 0]
        self.camera_fov = 55.371784673413806
        if self.play_only is False:
            self.camera_position[0] = self.camera_position[0] + random.uniform(-1, 1)*0.001
            self.camera_position[1] = self.camera_position[1] + random.uniform(-1, 1)*0.001
            self.camera_position[2] = self.camera_position[2] + random.uniform(-1, 1)*0.001
            self.camera_target[0] = self.camera_target[0] + random.uniform(-1, 1)*0.001
            self.camera_target[1] = self.camera_target[1] + random.uniform(-1, 1)*0.001
            self.camera_fov = 55.371784673413806 + random.uniform(-0.01, 0.01)
        if self.bc is True:
            self.camera_position = [0, 0, 0.25]
            self.camera_target = [0, 0, 0]
            self.camera_fov = 55.371784673413806

    def reset(self):
        self.steps = 0
        self.episode += 1
        self.current_objects = []
        self.target_objects = []
        self.build_environment()

        return self.get_observation()

    def step(self, action):

        done = False
        reward = 0
        self.steps += 1
        observation = None

        if self.discrete is True:
            # o_x = (action[0] * 2 - 100) * 0.001
            # o_y = (action[1] * 2 - 100) * 0.001
            o_x = (action[0] * 2 - 50) * 0.002
            o_y = (action[1] * 2 - 50) * 0.002
            o_yaw = action[2] * 45 - 180
        else:
            o_x = action[0] * 0.1
            o_y = action[1] * 0.1
            o_yaw = action[2] * 180

        if self.play_only is True:
            pos = [o_x, o_y, 0]
            ori = p.getQuaternionFromEuler([0, 0, math.radians(o_yaw)])
        else:
            pos = [o_x + random.uniform(-0.0005, 0.0005), o_y + random.uniform(-0.0005, 0.0005), 0]
            ori = p.getQuaternionFromEuler([0, 0, math.radians(o_yaw + random.uniform(-0.5, 0.5))])

        if self.pre_load is True:

            right_yaw = False

            if self.load_x + 1 == 1:
                tx1, ty1, _ = self.target_pos[0]
                tx2, ty2, _ = self.target_pos[1]
                if self.dist([o_x, o_y], [tx1, ty1]) < self.dist([o_x, o_y], [tx2, ty2]):
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[0])
                else:
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[1])
                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                else:
                    if abs(o_yaw - tyaw) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 2:
                if self.a == 1:
                    tx, ty, _ = self.target_pos[1]
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[1])
                elif self.a == 2:
                    tx, ty, _ = self.target_pos[0]
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[0])

                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                else:
                    if abs(o_yaw - tyaw) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 6:
                tx1, ty1, _ = self.target_pos[5]
                tx2, ty2, _ = self.target_pos[6]
                if self.dist([o_x, o_y], [tx1, ty1]) < self.dist([o_x, o_y], [tx2, ty2]):
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[5])
                else:
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[6])
                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                else:
                    if abs(o_yaw - tyaw) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 7:
                if self.a == 6:
                    tx, ty, _ = self.target_pos[6]
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[6])
                elif self.a == 7:
                    tx, ty, _ = self.target_pos[5]
                    _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[5])

                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                else:
                    if abs(o_yaw - tyaw) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 3:
                tx, ty, _ = self.target_pos[2]
                _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[2])
                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 90) < 20:
                    if abs(abs(o_yaw) - 90) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 0) < 20:
                    if abs(abs(o_yaw) - 0) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 45) < 20:
                    if abs(abs(o_yaw) - 45) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 135) < 20:
                    if abs(abs(o_yaw) - 135) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 4:
                tx, ty, _ = self.target_pos[3]
                _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[3])
                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 90) < 20:
                    if abs(abs(o_yaw) - 90) < 20:
                        right_yaw = True
                if abs(abs(tyaw) - 0) < 20:
                    if abs(abs(o_yaw) - 0) < 20:
                        right_yaw = True
                if abs(tyaw - 135) < 20 or abs(tyaw - -45) < 20:
                    if abs(o_yaw - 135) < 20 or abs(o_yaw - -45) < 20:
                        right_yaw = True
                if abs(tyaw - -135) < 20 or abs(tyaw - 45) < 20:
                    if abs(o_yaw - -135) < 20 or abs(o_yaw - 45) < 20:
                        right_yaw = True

            elif self.load_x + 1 == 5:
                tx, ty, _ = self.target_pos[4]
                _, _, tyaw = p.getEulerFromQuaternion(self.target_ori[4])
                tyaw = math.degrees(tyaw)
                if abs(abs(tyaw) - 180) < 20:
                    if abs(abs(o_yaw) - 180) < 20:
                        right_yaw = True
                else:
                    if abs(o_yaw - tyaw) < 20:
                        right_yaw = True

            c_uid = p.loadURDF('./model/'+str(self.load_x+1)+'.urdf', pos, ori)
            self.current_objects.append(c_uid)

            p.performCollisionDetection()
            self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            if right_yaw is True:
                reward = self.get_reward(c_uid, self.load_x+1)

            observation = self.get_observation()
            done = True

        elif self.pre_load is False or self.play_only is True:

            right_yaw = True

            if self.bc is True:
                pos = [action[0], action[1], 0]
                ori = p.getQuaternionFromEuler([0, 0, action[2]])
                c_uid = p.loadURDF('./model/'+str(self.steps)+'.urdf', pos, ori)
            else:
                c_uid = p.loadURDF('./model/'+str(self.steps)+'.urdf', pos, ori)
            self.current_objects.append(c_uid)

            p.performCollisionDetection()
            self.random_camera()
            self.current_image, _, self.current_mask = render_camera(self.camera_position, self.camera_target, self.camera_fov)

            # check collision
            if self.play_only is True:
                reward = self.get_reward(self.steps, self.steps)
            else:
                is_c = False
                for object_id in self.current_objects:
                    if is_c is True:
                        #done = True
                        reward = 0
                        break
                    for px in p.getContactPoints(object_id):
                        if px[2] != self.table_id and px[8] < -0.002:
                            is_c = True
                            break
                if is_c is False and right_yaw is True:
                    reward = self.get_reward(self.steps, self.steps)

            observation = self.get_observation()
            if self.steps == len(self.selected_object):
                done = True

        return observation, reward, done, {}

    def get_reward(self, object_index_in_mask, step):

        if step == 1:
            total_good_points = len(np.where(self.target_mask == 1)[0])
            current_good_points = len(
                np.where((self.target_mask == 1) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_1 = current_good_points/total_good_points

            total_good_points = len(np.where(self.target_mask == 2)[0])
            current_good_points = len(
                np.where((self.target_mask == 2) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_2 = current_good_points/total_good_points

            positive_reward = max(positive_reward_1, positive_reward_2)

        elif step == 2:
            total_good_points = len(np.where(self.target_mask == 1)[0])
            current_good_points = len(
                np.where((self.target_mask == 1) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_1 = current_good_points/total_good_points

            total_good_points = len(np.where(self.target_mask == 2)[0])
            current_good_points = len(
                np.where((self.target_mask == 2) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_2 = current_good_points/total_good_points

            positive_reward = max(positive_reward_1, positive_reward_2)

        elif step == 6:
            total_good_points = len(np.where(self.target_mask == 6)[0])
            current_good_points = len(
                np.where((self.target_mask == 6) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_1 = current_good_points/total_good_points

            total_good_points = len(np.where(self.target_mask == 7)[0])
            current_good_points = len(
                np.where((self.target_mask == 7) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_2 = current_good_points/total_good_points

            positive_reward = max(positive_reward_1, positive_reward_2)

        elif step == 7:
            total_good_points = len(np.where(self.target_mask == 6)[0])
            current_good_points = len(
                np.where((self.target_mask == 6) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_1 = current_good_points/total_good_points

            total_good_points = len(np.where(self.target_mask == 7)[0])
            current_good_points = len(
                np.where((self.target_mask == 7) & (self.current_mask == object_index_in_mask))[0])
            positive_reward_2 = current_good_points/total_good_points

            positive_reward = max(positive_reward_1, positive_reward_2)

        else:
            total_good_points = len(
                np.where(self.target_mask == object_index_in_mask)[0])
            current_good_points = len(np.where((self.target_mask == object_index_in_mask) & (
                self.current_mask == object_index_in_mask))[0])
            positive_reward = current_good_points/total_good_points

        if self.play_only is True:
            pass
        elif object_index_in_mask < 6:
            if positive_reward < 0.5:
                positive_reward = 0
            else:
                positive_reward = normalize_01(positive_reward, [0.5, 1])

        reward = positive_reward
        return reward

    def build_and_save_environment(self):
        self.random_camera()
        ground = False
        if ground is True:
            self.drop_ground_objects()
        else:
            while True:
                p.resetSimulation()
                self.target_objects = []
                self.table_id = p.loadURDF('./model/table.urdf', [0, 0, -0.02], useFixedBase=True)
                self.spawnobjects()
                self.delay(1)

                if_ok = True
                for object_id in self.target_objects:
                    if if_ok is False:
                        break
                    for px in p.getContactPoints(object_id):
                        if px[2] != self.table_id and px[8] < -0.0005:
                            if_ok = False
                if if_ok is True:
                    break

    def drop_ground_objects(self):
        force = -0.4
        self.selected_object = [1, 2, 3, 4, 5, 6, 7]
        while True:
            p.resetSimulation()
            p.setGravity(0, 0, -9.8)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())

            self.target_objects = []
            self.table_id = p.loadURDF('plane.urdf', [0, 0, 0], useFixedBase=True)
            for o_index in self.selected_object:
                pos = [random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), 0]
                yaw_random = random.choice([math.radians(-180), math.radians(-135), math.radians(-90), math.radians(-45),
                                           math.radians(0), math.radians(45), math.radians(90), math.radians(135), math.radians(180)])
                ori = p.getQuaternionFromEuler([0, 0, yaw_random])

                uid = p.loadURDF('./model/'+str(o_index)+'.urdf', pos, ori)
                p.changeDynamics(uid, -1, maxJointVelocity=1, spinningFriction=10e9)
                self.target_objects.append(uid)
            # Group the objects
            over_time = False
            count = 0
            n_contact_points = 0
            while n_contact_points < len(self.target_objects):
                n_contact_points = 0
                for target_object in self.target_objects:
                    # Check of the object is in contact with other objects
                    contact_points = [p for p in p.getContactPoints(target_object) if p[2] != self.table_id]
                    n_contact_points += 1 if len(contact_points) else 0

                    # Avoid putting to much of inertia on the objects
                    velocity_vector = p.getBaseVelocity(target_object)[0]
                    velocity = np.linalg.norm(velocity_vector)
                    if velocity > 0.2:
                        continue

                    # Apply attraction force
                    position = p.getBasePositionAndOrientation(target_object)[0]
                    force_vector = (
                        np.sign(position[0]) * force,
                        np.sign(position[1]) * force,
                        0
                    )
                    p.applyExternalForce(target_object, -1, force_vector, [0, 0, 0], p.WORLD_FRAME)
                p.stepSimulation()
                count += 1
                if count > 400:
                    over_time = True
                    break

            if over_time is False:
                if_ok = True
                for object_id in self.target_objects:
                    pos, _ = p.getBasePositionAndOrientation(object_id)
                    if -0.1 < pos[0] < 0.1 and -0.1 < pos[1] < 0.1 and pos[2] < -0.092:
                        pass
                    else:
                        if_ok = False
                        break

                if if_ok is True:
                    break
            else:
                pass

    def dist(self, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2
        return (x2 - x1)**2 + (y2 - y1)**2
