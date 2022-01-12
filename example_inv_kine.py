import numpy as np
from pinocchio import neutral, SE3
from pinocchio.utils import eye
from legged_robot import Robot
import time
from inverse_kinematics import InverseKinematics

robot = Robot()
robot.viewer.viewer.gui.setVisibility('world/floor', 'OFF')
q_init = neutral(robot.model)
robot.display(q_init)

time.sleep(4)

ik = InverseKinematics(robot)
leftFootTranslation = np.array([-.5, -0.3, -1.7])
rightFootTranslation = np.array([.5, .2, -2.1])

ik.leftFootRefPose = SE3(eye(3), leftFootTranslation)
ik.rightFootRefPose = SE3(eye(3), rightFootTranslation)
ik.waistRefPose = SE3(eye(3), np.array([0, 0., 0.]))

q_result = ik.solve(q_init)

anime = 1000
dt = 1/anime
q_diff = dt*(q_result - q_init)
q = q_init
for i in range(anime):
    q += q_diff
    robot.display(q)
    time.sleep(dt)
robot.display(q_result)
