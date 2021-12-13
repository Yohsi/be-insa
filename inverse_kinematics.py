import numpy as np
import numpy.linalg
from scipy.optimize import fmin_bfgs
from pinocchio.utils import *
from pinocchio import SE3, forwardKinematics, log, neutral
import eigenpy
from legged_robot import Robot
eigenpy.switchToNumpyMatrix()


class CallbackLogger:
    def __init__(self, ik):
        self.nfeval = 1
        self.ik = ik

    def __call__(self, x):
        print('===CBK=== {0:4d}   {1}'.format(self.nfeval, self.ik.latestCost))
        self.nfeval += 1


class InverseKinematics(object):
    def __init__(self, robot):
        self.q = neutral(robot.model)
        forwardKinematics(robot.model, robot.data, self.q)
        self.robot = robot
        # Initialize references of feet and center of mass with initial values
        self.leftFootRefPose = robot.data.oMi[robot.leftFootJointId].copy()
        self.rightFootRefPose = robot.data.oMi[robot.rightFootJointId].copy()
        self.waistRefPose = robot.data.oMi[robot.waistJointId].copy()
        self.latestCost = 0

    def cost(self, q):
        forwardKinematics(self.robot.model, self.robot.data, q)
        robot = self.robot
        leftFootPose = robot.data.oMi[robot.leftFootJointId].copy()
        rightFootPose = robot.data.oMi[robot.rightFootJointId].copy()
        waistPose = robot.data.oMi[robot.waistJointId].copy()
        ssq = (self.leftFootRefPose.translation[0][0] - leftFootPose.translation[0][0])**2
        ssq += (self.leftFootRefPose.translation[1][0] - leftFootPose.translation[1][0])**2
        ssq += (self.leftFootRefPose.translation[2][0] - leftFootPose.translation[2][0])**2
        ssq += (self.rightFootRefPose.translation[0][0] - rightFootPose.translation[0][0])**2
        ssq += (self.rightFootRefPose.translation[1][0] - rightFootPose.translation[1][0])**2
        ssq += (self.rightFootRefPose.translation[2][0] - rightFootPose.translation[2][0])**2
        ssq += (self.waistRefPose.translation[0][0] - waistPose.translation[0][0])**2
        ssq += (self.waistRefPose.translation[1][0] - waistPose.translation[1][0])**2
        ssq += (self.waistRefPose.translation[2][0] - waistPose.translation[2][0])**2
        return ssq


    def solve(self, q_init):
        bfgs_result = fmin_bfgs(self.cost, q_init, callback=CallbackLogger(self))
        self.latestCost = bfgs_result
        return bfgs_result


robot = Robot()
a = InverseKinematics(robot)
a.leftFootRefPose = SE3(eye(3), np.array([0, 0., -1]))
a.rightFootRefPose = SE3(eye(3), np.array([0, 0., -1]))
a.waistRefPose = SE3(eye(3), np.array([0, 0., 0.]))
print(a.solve(neutral(robot.model)))
