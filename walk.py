import time
import numpy as np
from pinocchio import neutral, SE3
from pinocchio.utils import eye
from legged_robot import Robot
from inverse_kinematics import InverseKinematics


class PiecewiseLinear:
    def __init__(self, x_init, y_init, z_init):
        self.times = [0]
        self.segments = [np.array([x_init, y_init, z_init])]

    def add_segment(self, x, y, z, duration):
        self.times.append(self.times[-1] + duration)
        self.segments.append(np.array([x, y, z]))

    def eval(self, t):
        # print([round(i, 1) for i in self.times])
        # print(self.segments)
        if t < 0:
            t = 0
        if t > self.times[-1]:
            t = self.times[-1]

        i = 0
        while t > self.times[i]:
            i += 1

        start = self.segments[i - 1]
        end = self.segments[i]
        t_start = self.times[i - 1]
        t_end = self.times[i]

        k = (t - t_start) / (t_end - t_start)
        return start + k * (end - start)


class PiecewisePolynomial:
    pass


# def move_to_coord(leftFoot, rightFoot, waist, duration):
#     global q_prev
#     ik = InverseKinematics(robot)
#     ik.leftFootRefPose = SE3(eye(3), leftFoot)
#     ik.rightFootRefPose = SE3(eye(3), rightFoot)
#     ik.waistRefPose = SE3(eye(3), waist)

#     q_result = ik.solve(q_prev)

#     dt = 1 / 60
#     nb_iter = int(duration / dt)

#     q_diff = (q_result - q_prev) / nb_iter
#     q = q_prev
#     for i in range(nb_iter):
#         q += q_diff
#         robot.display(q)
#         time.sleep(dt)

#     robot.display(q_result)
#     q_prev = q_result


robot = Robot()
robot.viewer.viewer.gui.setVisibility('world/floor', 'OFF')
q_init = neutral(robot.model)
q_init[6, 0] = 0.1
q_init[12, 0] = 0.1
q_prev = q_init
robot.display(q_init)

z_pied_leve = -1.9
z_pied_baisse = -2.1

posesLeft = PiecewiseLinear(-0.3, 0, z_pied_baisse)
posesRight = PiecewiseLinear(0.3, 0, z_pied_baisse)
posesWaist = PiecewiseLinear(0, 0, 0)

dt = 0.6
t = dt

# on décale le bassin vers la gauche
posesLeft.add_segment(-0.3, 0, z_pied_baisse, dt)
posesRight.add_segment(0.3, 0, z_pied_baisse, dt)
posesWaist.add_segment(-0.3, 0, 0, dt)

# on monte le pied droit
posesLeft.add_segment(-0.3, 0, z_pied_baisse, dt)
posesRight.add_segment(0.3, 0, z_pied_leve, dt)
posesWaist.add_segment(-0.3, 0, 0, dt)

# on avance le pied droit
posesLeft.add_segment(-0.3, 0, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_leve, dt)
posesWaist.add_segment(-0.3, 0, 0, dt)

# on descend le pied droit
posesLeft.add_segment(-0.3, 0, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(-0.3, 0, 0, dt)

# on place le centre de gravité sur le pied droit
posesLeft.add_segment(-0.3, 0, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -0.5, 0, dt)

# on monte le pied gauche
posesLeft.add_segment(-0.3, 0, z_pied_leve, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -0.5, 0, dt)

# on avance le pied gauche
posesLeft.add_segment(-0.3, -1, z_pied_leve, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -0.5, 0, dt)

# on descend le pied gauche
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -0.5, 0, dt)

# on place le centre de gravité sur le pied gauche
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_baisse, dt)
posesWaist.add_segment(-0.3, -1, 0, dt)

# on monte le pied droit
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -0.5, z_pied_leve, dt)
posesWaist.add_segment(-0.3, -1, 0, dt)

# on avance le pied droit
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -1.5, z_pied_leve, dt)
posesWaist.add_segment(-0.3, -1, 0, dt)

# on descend le pied droit
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(-0.3, -1, 0, dt)

# on place le centre de gravité sur le pied droit
posesLeft.add_segment(-0.3, -1, z_pied_baisse, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -1.5, 0, dt)

# on monte le pied gauche
posesLeft.add_segment(-0.3, -1, z_pied_leve, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -1.5, 0, dt)

# on avance le pied gauche
posesLeft.add_segment(-0.3, -1.5, z_pied_leve, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -1.5, 0, dt)

# on descend le pied gauche
posesLeft.add_segment(-0.3, -1.5, z_pied_baisse, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(0.3, -1.5, 0, dt)

# on replace le centre de gravité
posesLeft.add_segment(-0.3, -1.5, z_pied_baisse, dt)
posesRight.add_segment(0.3, -1.5, z_pied_baisse, dt)
posesWaist.add_segment(0, -1.5, 0, dt)

# prevTime = 0
# for t in posesWaist.keys():
#     duration = t - prevTime
#     prevTime = t

increment = 0.05
q_prev = q_init
for t in np.arange(0, 10, increment):
    ik = InverseKinematics(robot)
    ik.leftFootRefPose = SE3(eye(3), posesLeft.eval(t))
    ik.rightFootRefPose = SE3(eye(3), posesRight.eval(t))
    ik.waistRefPose = SE3(eye(3), posesWaist.eval(t))
    q_result = ik.solve(q_prev)
    q_prev = q_result
    robot.display(q_result)
    time.sleep(increment)
    # print([round(i, 1) for i in posesLeft.eval(t)])
