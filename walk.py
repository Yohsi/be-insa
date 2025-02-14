import time
import numpy as np
from pinocchio import neutral, SE3
from pinocchio.utils import eye
from legged_robot import Robot
from inverse_kinematics import InverseKinematics
from scipy.interpolate import lagrange

class PiecewiseLinear:
    def __init__(self, x_init, y_init, z_init):
        self.times = [0]
        self.segments = [np.array([x_init, y_init, z_init])]

    def add_segment(self, x, y, z, duration):
        self.times.append(self.times[-1] + duration)
        self.segments.append(np.array([x, y, z]))

    def max_t(self):
        return self.times[-1]

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
    def __init__(self, x_init, y_init, z_init):
        self.times = [0]
        self.segments = [np.array([x_init, y_init, z_init])]

    def add_segment(self, x, y, z, duration):
        self.times.append(self.times[-1] + duration)
        self.segments.append(np.array([x, y, z]))

    def max_t(self):
        return self.times[-1]

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

        if i == 0: 
            return self.segments[0]
        
        elif i == 1:
            px = [self.segments[i - 1][0], self.segments[i][0], self.segments[i + 1][0]]
            py = [self.segments[i - 1][1], self.segments[i][1], self.segments[i + 1][1]]
            pz = [self.segments[i - 1][2], self.segments[i][2], self.segments[i + 1][2]]
            pt = [self.times[i - 1], self.times[i], self.times[i + 1]]

        elif i == len(self.times) - 1:
            px = [self.segments[i - 2][0], self.segments[i - 1][0], self.segments[i][0]]
            py = [self.segments[i - 2][1], self.segments[i - 1][1], self.segments[i][1]]
            pz = [self.segments[i - 2][2], self.segments[i - 1][2], self.segments[i][2]]
            pt = [self.times[i - 2], self.times[i - 1], self.times[i]]

        else:
            px = [self.segments[i - 2][0], self.segments[i - 1][0], self.segments[i][0], self.segments[i + 1][0]]
            py = [self.segments[i - 2][1], self.segments[i - 1][1], self.segments[i][1], self.segments[i + 1][1]]
            pz = [self.segments[i - 2][2], self.segments[i - 1][2], self.segments[i][2], self.segments[i + 1][2]]
            pt = [self.times[i - 2], self.times[i - 1], self.times[i], self.times[i + 1]]

        fx = lagrange(pt,px)
        fy = lagrange(pt,py)
        fz = lagrange(pt,pz)

        return np.array([fx(t), fy(t), fz(t)])


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

posesLeft = PiecewisePolynomial(-0.3, 0, z_pied_baisse)
posesRight = PiecewisePolynomial(0.3, 0, z_pied_baisse)
posesWaist = PiecewisePolynomial(0, 0, 0)

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

# calcul de toutes les positions articulaires avant affichage

increment = 0.1
dt = 1 / 60
nb_iter = int(increment / dt)

qs = [q_init]

print("0%", end="")

for t in np.arange(0, posesLeft.max_t(), increment):
    ik = InverseKinematics(robot)
    ik.leftFootRefPose = SE3(eye(3), posesLeft.eval(t))
    ik.rightFootRefPose = SE3(eye(3), posesRight.eval(t))
    ik.waistRefPose = SE3(eye(3), posesWaist.eval(t))
    qs.append(ik.solve(qs[-1]))
    print(f"\r{int(t / posesLeft.max_t() * 100)}%", end="")

print("\r100%")


for i in range(0, len(qs)-1):
    q_diff = (qs[i+1] - qs[i]) / nb_iter
    q = qs[i]
    for j in range(nb_iter):
        q += q_diff
        robot.display(q)
        time.sleep(dt)
    robot.display(qs[i+1])
