import time
import socket
import urx
import math
import os
import math3d as m3d
import serial
import serial.tools.list_ports

Camera_x_offset = -0.17281 - 0.002
Camera_y_offset = -0.0025


class RealRobot:
    def __init__(self, table=True, chuan=False, has_gripper=True):

        self.robot = urx.Robot("192.168.1.102", use_rt=True)

        if has_gripper is True:
            self.serialcomm = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        self.X_OFFSET = 0.6
        self.Y_OFFSET = 0.3
        self.Z_OFFSET = 0.3 - 0.038

    def close_gripper(self):
        self.serialcomm.write(b's')

    def open_gripper(self):
        self.serialcomm.write(b'f')

    def go_r_home(self, acc, vel):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        # self.move_world(self.X_OFFSET, self.Y_OFFSET, self.Z_OFFSET, math.radians(127.28), math.radians(127.29), 0, acc, vel)
        self.move_world(self.X_OFFSET, self.Y_OFFSET,
                        self.Z_OFFSET, math.pi, 0, 0, acc, vel)

    def go_q_home(self, acc, vel):
        self.move_joint(home_position=[19.51,-93.86,-128.34,-47.82,-270.44,-69.76], acc=acc, vel=vel, wait = True)
        self.set_tcp(0, 0, 0, 0, 0, 0)
        # self.move_world(self.X_OFFSET, self.Y_OFFSET-0.25,
        #                 self.Z_OFFSET, math.pi, 0, 0, acc, vel)

    def go_c_home(self, acc, vel):
        self.move_joint(home_position=[52.73, -85.67, -137.05, -45.51, -272.71, -36.60], acc=acc, vel=vel, wait = True)
        self.set_tcp(0, 0, 0, 0, 0, 0)
        # self.move_world(self.X_OFFSET+Camera_x_offset, self.Y_OFFSET +
        #                 Camera_y_offset, self.Z_OFFSET, math.pi, 0, 0, acc, vel)
        # self.set_gripper_ori(roll=math.radians(-1), pitch=math.radians(
        #    3.5), yaw=0, acc=0.1, vel=0.1, wait=True)

    def move_joint(self, home_position, acc=0.025, vel=0.01, wait=True):
        Hong_joint0 = math.radians(home_position[0])
        Hong_joint1 = math.radians(home_position[1])
        Hong_joint2 = math.radians(home_position[2])
        Hong_joint3 = math.radians(home_position[3])
        Hong_joint4 = math.radians(home_position[4])
        Hong_joint5 = math.radians(home_position[5])
        self.robot.movej((Hong_joint0, Hong_joint1, Hong_joint2,
                         Hong_joint3, Hong_joint4, Hong_joint5,), acc*3, vel*3, wait=wait)

    def set_tcp(self, x, y, z, rx, ry, rz):
        self.robot.set_tcp((x, y, z, rx, ry, rz))
        time.sleep(1)

    def get_tool_position(self):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - self.X_OFFSET
        y = tool_pose[1] - self.Y_OFFSET
        z = tool_pose[2]
        return (x, y, z)

    def cal_furure(self, targetxyz):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - self.X_OFFSET
        y = tool_pose[1] - self.Y_OFFSET
        z = tool_pose[2]
        rel_x = targetxyz[0] - x
        rel_y = targetxyz[1] - y
        rel_z = targetxyz[2] - z
        return rel_x, rel_y, rel_z

    def move_r_with_ori(self, targetxyz, acc=0.025, vel=0.01, wait=True, relative=True):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - self.X_OFFSET
        y = tool_pose[1] - self.Y_OFFSET
        z = tool_pose[2] - self.Z_OFFSET
        rela_x = targetxyz[0] - x
        rela_y = targetxyz[1] - y
        rela_z = targetxyz[2] - z

        if wait is True:
            self.robot.movel((rela_x, rela_y, rela_z, 0, 0, 0),
                             acc, vel, wait, relative)
        else:
            self.robot = urx.Robot("192.168.1.102", use_rt=True)
            self.robot.movel((rela_x, rela_y, rela_z, 0, 0, 0),
                             acc, vel, wait, relative)

            count = 0
            time.sleep(0.5)
            while self.robot.get_force() < 45:
                time.sleep(0.001)
                print(self.robot.get_force(), self.robot.is_program_running())
                if self.robot.is_program_running() is False:
                    count += 1
                    if count > 1:
                        print('large force')
                        break
            self.robot.stopl()

    def move_q_with_ori(self, targetxyz, acc=0.025, vel=0.01, wait=True, relative=True):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - self.X_OFFSET
        y = tool_pose[1] - self.Y_OFFSET + 0.25
        z = tool_pose[2] - self.Z_OFFSET
        rela_x = targetxyz[0] - x
        rela_y = targetxyz[1] - y
        rela_z = targetxyz[2] - z
        self.robot.movel((rela_x, rela_y, rela_z, 0, 0, 0),
                         acc, vel, wait, relative)

    def move_world(self, wx, wy, wz, rx, ry, rz, acc=0.025, vel=0.01, wait=True, relative=True):
        self.robot.movel((wx, wy, wz, rx, ry, rz), acc, vel)

    def set_gripper_ori(self, roll, pitch, yaw, acc=0.1, vel=0.1, wait=True):
        rot_acc = acc*2
        rot_vel = vel*2
        move = m3d.Transform((0, 0, 0, 0, 0, -yaw))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=wait, command="movel", threshold=None)
        move = m3d.Transform((0, 0, 0, roll, 0, 0))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=wait, command="movel", threshold=None)
        move = m3d.Transform((0, 0, 0, 0, pitch, 0))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=wait, command="movel", threshold=None)

    def apply_action_on_real_robot(self, step, o_x, o_y, o_yaw, acc=0.1, vel=0.1, pick = True):
        acc = acc
        vel = vel
        changed_yaw = None
        if pick is True:
            self.go_q_home(acc, vel)
            self.close_gripper()
            if step == 1:
                self.move_q_with_ori([0, -0.025, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([0, -0.025, 0], acc=acc, vel=vel)
            if step == 2:
                self.move_q_with_ori([-0.025, 0, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([-0.025, 0, 0], acc=acc, vel=vel)
            if step == 3:
                self.move_q_with_ori([0.025, 0, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([0.025, 0, 0], acc=acc, vel=vel)
            if step == 4:
                self.move_q_with_ori([-0.0125, 0.0375, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([-0.0125, 0.0375, 0], acc=acc, vel=vel)
            if step == 5:
                self.move_q_with_ori([0.0375, 0.0375, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([0.0375, 0.0375, 0], acc=acc, vel=vel)
            if step == 6:
                self.move_q_with_ori([0.0375, -0.025, -0.185], acc=acc, vel=vel)
                time.sleep(1.01)
                self.move_q_with_ori([0.0375, -0.025, 0], acc=acc, vel=vel)
            if step == 7:
                self.move_q_with_ori([0, 0.0125, -0.185], acc=acc, vel=acc)
                time.sleep(1.01)
                self.move_q_with_ori([0, 0.0125, 0], acc=acc, vel=acc)
            self.go_c_home(acc, vel)
        else:
            if step == 1:
                self.go_r_home(acc, vel)
                if o_yaw == 0:
                    self.set_gripper_ori(
                        0, 0, math.radians(-135), acc=acc, vel=vel, wait=True)
                    changed_yaw = -135
                elif o_yaw == 45:
                    self.set_gripper_ori(
                        0, 0, math.radians(-90), acc=acc, vel=vel, wait=True)
                    changed_yaw = -90
                elif o_yaw == 90:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
                elif o_yaw == 135:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == -45:
                    self.set_gripper_ori(0, 0, math.radians(
                        180), acc=acc, vel=vel, wait=True)
                    changed_yaw = 180
                elif o_yaw == -90:
                    self.set_gripper_ori(0, 0, math.radians(
                        135), acc=acc, vel=vel, wait=True)
                    changed_yaw = 135
                elif o_yaw == -135:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
                elif o_yaw == -180 or o_yaw == 180:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
            if step == 2:
                self.go_r_home(acc, vel)
                if o_yaw == 0:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
                elif o_yaw == 45:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == 90:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
                elif o_yaw == 135:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
                elif o_yaw == -45:
                    self.set_gripper_ori(
                        0, 0, math.radians(-90), acc=acc, vel=vel, wait=True)
                    changed_yaw = -90
                elif o_yaw == -90:
                    self.set_gripper_ori(
                        0, 0, math.radians(-135), acc=acc, vel=vel, wait=True)
                    changed_yaw = -135
                elif o_yaw == -135:
                    self.set_gripper_ori(
                        0, 0, math.radians(-180), acc=acc, vel=vel, wait=True)
                    changed_yaw = -180
                elif o_yaw == -180 or o_yaw == 180:
                    self.set_gripper_ori(0, 0, math.radians(
                        135), acc=acc, vel=vel, wait=True)
                    changed_yaw = 135
            if step == 3:
                self.go_r_home(acc, vel)
                if o_yaw == 0 or o_yaw == 90 or o_yaw == -90 or o_yaw == 180 or o_yaw == -180:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
                elif o_yaw == 45 or o_yaw == 135 or o_yaw == -45 or o_yaw == -135:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
            if step == 4:
                self.go_r_home(acc, vel)
                if o_yaw == 0 or o_yaw == 180 or o_yaw == -180:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
                elif o_yaw == 90 or o_yaw == -90:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == 45 or o_yaw == -135:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
                elif o_yaw == -45 or o_yaw == 135:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
            if step == 5:
                self.go_r_home(acc, vel)
                if o_yaw == 0:
                    self.set_gripper_ori(
                        0, 0, math.radians(-90), acc=acc, vel=vel, wait=True)
                    changed_yaw = -90
                elif o_yaw == 45:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
                elif o_yaw == 90:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == 135:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
                elif o_yaw == -45:
                    self.set_gripper_ori(
                        0, 0, math.radians(-135), acc=acc, vel=vel, wait=True)
                    changed_yaw = -135
                elif o_yaw == -90:
                    self.set_gripper_ori(0, 0, math.radians(
                        180), acc=acc, vel=vel, wait=True)
                    changed_yaw = 180
                elif o_yaw == -135:
                    self.set_gripper_ori(0, 0, math.radians(
                        135), acc=acc, vel=vel, wait=True)
                    changed_yaw = 135
                elif o_yaw == -180 or o_yaw == 180:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
            if step == 6:
                self.go_r_home(acc, vel)
                if o_yaw == 0:
                    self.set_gripper_ori(0, 0, math.radians(
                        135), acc=acc, vel=vel, wait=True)
                    changed_yaw = 135
                elif o_yaw == 45:
                    self.set_gripper_ori(
                        0, 0, math.radians(-180), acc=acc, vel=vel, wait=True)
                    changed_yaw = -180
                elif o_yaw == 90:
                    self.set_gripper_ori(
                        0, 0, math.radians(-135), acc=acc, vel=vel, wait=True)
                    changed_yaw = -135
                elif o_yaw == 135:
                    self.set_gripper_ori(
                        0, 0, math.radians(-90), acc=acc, vel=vel, wait=True)
                    changed_yaw = -90
                elif o_yaw == -45:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
                elif o_yaw == -90:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
                elif o_yaw == -135:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == -180 or o_yaw == 180:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
            if step == 7:
                self.go_r_home(acc, vel)
                if o_yaw == 0:
                    self.set_gripper_ori(0, 0, math.radians(
                        45), acc=acc, vel=vel, wait=True)
                    changed_yaw = 45
                elif o_yaw == 45:
                    self.set_gripper_ori(0, 0, math.radians(
                        90), acc=acc, vel=vel, wait=True)
                    changed_yaw = 90
                elif o_yaw == 90:
                    self.set_gripper_ori(0, 0, math.radians(
                        135), acc=acc, vel=vel, wait=True)
                    changed_yaw = 135
                elif o_yaw == 135:
                    self.set_gripper_ori(
                        0, 0, math.radians(-180), acc=acc, vel=vel, wait=True)
                    changed_yaw = -180
                elif o_yaw == -45:
                    self.set_gripper_ori(0, 0, math.radians(
                        0), acc=acc, vel=vel, wait=True)
                    changed_yaw = 0
                elif o_yaw == -90:
                    self.set_gripper_ori(
                        0, 0, math.radians(-45), acc=acc, vel=vel, wait=True)
                    changed_yaw = -45
                elif o_yaw == -135:
                    self.set_gripper_ori(
                        0, 0, math.radians(-90), acc=acc, vel=vel, wait=True)
                    changed_yaw = -90
                elif o_yaw == -180 or o_yaw == 180:
                    self.set_gripper_ori(
                        0, 0, math.radians(-135), acc=acc, vel=vel, wait=True)
                    changed_yaw = -135
            self.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
            self.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
            self.open_gripper()
            self.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)

    def get_rob(self):
        pass
        return self.robot


# %%
if __name__ == '__main__':
    rob = RealRobot(has_gripper=True)
    rob.set_tcp(0, 0, 0, 0, 0, 0)
    robot = rob.get_rob()
    rob.go_c_home(0.1, 0.1)
    acc = 0.1
    vel = 0.1
    # 
    rob.go_q_home(acc, vel)
    rob.close_gripper()
    
    rob.move_q_with_ori([0, 0, -0.185], acc=acc, vel=vel)
    time.sleep(1.01)
    rob.move_q_with_ori([0, 0, 0], acc=acc, vel=vel)
    
    rob.go_c_home(0.1, 0.1)
    rob.go_r_home(acc, vel)
    rob.set_gripper_ori(0, 0, math.radians(0), acc=acc, vel=vel, wait=True)
    o_x = -0.035
    o_y = 0
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
    rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
    rob.open_gripper()
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
    # 
    rob.go_q_home(acc, vel)
    rob.close_gripper()
    
    rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)
    rob.move_q_with_ori([0.065, 0, -0.192], acc=acc, vel=vel)
    time.sleep(1.5)
    rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)
    
    rob.go_c_home(0.1, 0.1)
    rob.go_r_home(acc, vel)
    rob.set_gripper_ori(0, 0, math.radians(45), acc=acc, vel=vel, wait=True)
    o_x = 0.06
    o_y = -0.08
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
    rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
    rob.open_gripper()
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
    #    
    rob.go_q_home(acc, vel)
    rob.close_gripper()
    
    rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)
    rob.move_q_with_ori([-0.065, 0, -0.188], acc=acc, vel=vel)
    time.sleep(1.5)
    rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)
    
    rob.go_c_home(0.1, 0.1)
    rob.go_r_home(acc, vel)
    rob.set_gripper_ori(0, 0, math.radians(135), acc=acc, vel=vel, wait=True)
    o_x = 0.06
    o_y = 0.08
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
    rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
    rob.open_gripper()
    rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
    
    rob.move_joint(home_position=[80.28,-65.53,-150.07,-53.95,-272.93, -9.05], acc=0.2, vel=0.2, wait = True)
    rob.set_tcp(0, 0, 0, 0, 0, 0)
# %% 
# if __name__ == '__main__':
#     rob = RealRobot(has_gripper=True)
#     rob.set_tcp(0, 0, 0, 0, 0, 0)
#     robot = rob.get_rob()
#     rob.go_c_home(0.1, 0.1)
#     acc = 0.1
#     vel = 0.1
#     # 
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([0, 0, -0.185], acc=acc, vel=vel)
#     time.sleep(1.01)
#     rob.move_q_with_ori([0, 0, 0], acc=acc, vel=vel)
    
#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(0), acc=acc, vel=vel, wait=True)
#     o_x = 0
#     o_y = 0
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
#     # 
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)
#     rob.move_q_with_ori([0.065, 0, -0.192], acc=acc, vel=vel)
#     time.sleep(1.5)
#     rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)
    
#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(90), acc=acc, vel=vel, wait=True)
#     o_x = 0
#     o_y = 0.07
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
#     #    
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)
#     rob.move_q_with_ori([-0.065, 0, -0.188], acc=acc, vel=vel)
#     time.sleep(1.5)
#     rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)
    
#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(90), acc=acc, vel=vel, wait=True)
#     o_x = 0
#     o_y = -0.075
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
    
#     rob.move_joint(home_position=[80.28,-65.53,-150.07,-53.95,-272.93, -9.05], acc=0.2, vel=0.2, wait = True)
#     rob.set_tcp(0, 0, 0, 0, 0, 0)
# %%
# if __name__ == '__main__':
#     rob = RealRobot(has_gripper=True)
#     rob.set_tcp(0, 0, 0, 0, 0, 0)
#     robot = rob.get_rob()
#     rob.go_c_home(0.1, 0.1)
#     acc = 0.1
#     vel = 0.1
#     # 
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([0, 0, -0.185], acc=acc, vel=vel)
#     time.sleep(1.01)
#     rob.move_q_with_ori([0, 0, 0], acc=acc, vel=vel)

#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(0), acc=acc, vel=vel, wait=True)
#     o_x = 0
#     o_y = 0
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
#     # 
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)
#     rob.move_q_with_ori([0.065, 0, -0.192], acc=acc, vel=vel)
#     time.sleep(1.5)
#     rob.move_q_with_ori([0.065, 0, 0], acc=acc, vel=vel)

#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(90), acc=acc, vel=vel, wait=True)
#     o_x = 0.015
#     o_y = -0.08
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
#     #    
#     rob.go_q_home(acc, vel)
#     rob.close_gripper()
    
#     rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)
#     rob.move_q_with_ori([-0.065, 0, -0.188], acc=acc, vel=vel)
#     time.sleep(1.5)
#     rob.move_q_with_ori([-0.065, 0, 0], acc=acc, vel=vel)

#     rob.go_c_home(0.1, 0.1)
#     rob.go_r_home(acc, vel)
#     rob.set_gripper_ori(0, 0, math.radians(90), acc=acc, vel=vel, wait=True)
#     o_x = -0.02
#     o_y = 0.075
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel, wait=True)
#     rob.move_r_with_ori([o_x, o_y, -0.187], acc=acc, vel=vel, wait=False)
#     rob.open_gripper()
#     rob.move_r_with_ori([o_x, o_y, 0], acc=acc, vel=vel)
    
#     rob.move_joint(home_position=[80.28,-65.53,-150.07,-53.95,-272.93, -9.05], acc=0.2, vel=0.2, wait = True)
#     rob.set_tcp(0, 0, 0, 0, 0, 0)