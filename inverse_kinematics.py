import eigenpy
from pinocchio import forwardKinematics, neutral
from pinocchio.utils import *
from scipy.optimize import fmin_bfgs

eigenpy.switchToNumpyMatrix()


class CallbackLogger:
    def __init__(self, ik):
        self.nfeval = 1
        self.ik = ik

    def __call__(self, x):
        # print('===CBK=== {0:4d}   {1}'.format(self.nfeval, self.ik.latestCost))
        self.nfeval += 1


class InverseKinematics(object):
    def __init__(self, robot):
        q = neutral(robot.model)
        forwardKinematics(robot.model, robot.data, q)
        self.robot = robot
        # Initialize references of feet and center of mass with initial values
        self.leftFootRefPose = robot.data.oMi[robot.leftFootJointId].copy()
        self.rightFootRefPose = robot.data.oMi[robot.rightFootJointId].copy()
        self.waistRefPose = robot.data.oMi[robot.waistJointId].copy()
        self.latestCost = 0

    def cost(self, q):
        forwardKinematics(self.robot.model, self.robot.data, q)

        leftFootPose = self.robot.data.oMi[self.robot.leftFootJointId].copy()
        rightFootPose = self.robot.data.oMi[self.robot.rightFootJointId].copy()
        waistPose = self.robot.data.oMi[self.robot.waistJointId].copy()

        diffLeftFoot = self.leftFootRefPose.translation - leftFootPose.translation
        diffRightFoot = self.rightFootRefPose.translation - rightFootPose.translation
        diffWaist = self.waistRefPose.translation - waistPose.translation
        diffLeftFootRot = self.leftFootRefPose.rotation - leftFootPose.rotation
        diffRightFootRot = self.rightFootRefPose.rotation - rightFootPose.rotation

        ssq = np.sum(np.square(diffLeftFoot))
        ssq += np.sum(np.square(diffRightFoot))
        ssq += np.sum(np.square(diffWaist))
        ssq += np.sum(np.square(diffLeftFootRot))
        ssq += np.sum(np.square(diffRightFootRot))
        self.latestCost = ssq
        return ssq

    def solve(self, q_init):
        bfgs_result = fmin_bfgs(self.cost, q_init)
        return bfgs_result.reshape(15, 1)
