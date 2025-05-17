import pybullet as p
import pybullet_data
import random
import time
import numpy as np
import math


def main():

    force = -0.4
    models = [
        './model/1.urdf',
        './model/2.urdf',
        './model/3.urdf',
        './model/4.urdf',
        './model/5.urdf',
        './model/6.urdf',
        './model/7.urdf',
    ]
    objects = []
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_object = p.loadURDF('plane.urdf')
    p.setGravity(0, 0, -9.8)

    # self.selected_object = [1, 2, 3, 4, 5, 6, 7]
    selected_object = [1, 2]
    for o_index in selected_object:
        pos = [random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3), 0]
        yaw_random = random.choice([math.radians(-180), math.radians(-135), math.radians(-90), math.radians(-45),
                                   math.radians(0), math.radians(45), math.radians(90), math.radians(135), math.radians(180)])
        ori = p.getQuaternionFromEuler([0, 0, yaw_random])

        uid = p.loadURDF('./model/'+str(o_index)+'.urdf', pos, ori)
        p.changeDynamics(uid, -1, maxJointVelocity=1, spinningFriction=10e9)
        objects.append(uid)

    # Group the objects
    n_contact_points = 0
    while n_contact_points < len(objects):
        n_contact_points = 0
        for object in objects:
            # Check of the object is in contact with other objects
            contact_points = [p for p in p.getContactPoints(object) if p[2] != plane_object]
            n_contact_points += 1 if len(contact_points) else 0

            # Avoid putting to much of inertia on the objects
            velocity_vector = p.getBaseVelocity(object)[0]
            velocity = np.linalg.norm(velocity_vector)
            if velocity > 0.2:
                continue

            # Apply attraction force
            position = p.getBasePositionAndOrientation(object)[0]
            force_vector = (
                np.sign(position[0]) * force,
                np.sign(position[1]) * force,
                0
            )
            p.applyExternalForce(object, -1, force_vector, [0, 0, 0], p.WORLD_FRAME)
        p.stepSimulation()
    print('all objects are grouped')


if __name__ == '__main__':
    main()
