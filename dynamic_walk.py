import time
import numpy as np
from pinocchio import neutral, SE3
from pinocchio.utils import eye
from scipy.optimize.optimize import fmin_bfgs
from legged_robot import Robot
from inverse_kinematics import InverseKinematics
from scipy.interpolate import lagrange

step_dist = 1.4
dt = 0.3
G = 140


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


def cp(xy, increment, h):
    vx = np.diff(xy[0], prepend=[0])/increment
    ax = np.diff(vx, prepend=[0])/increment
    vy = np.diff(xy[1], prepend=[0])/increment
    ay = np.diff(vy, prepend=[0])/increment
    return xy - (h / G) * np.array([ax, ay])


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

com = []
t_base = []

def do_step(moving_foot, other_foot, x, moving_y, other_y, distance):
    new_y = moving_y - distance

    # on décale le bassin
    other_foot.add_segment(-x, other_y, z_pied_baisse, dt)
    moving_foot.add_segment(x, moving_y, z_pied_baisse, dt)
    com.append(np.array([-x, other_y]))
    t_base.append(dt)

    # on monte le pied 
    other_foot.add_segment(-x, other_y, z_pied_baisse, dt)
    moving_foot.add_segment(x, moving_y, z_pied_leve, dt)
    com.append(np.array([-x, other_y]))
    t_base.append(dt)

    # on avance le pied
    other_foot.add_segment(-x, other_y, z_pied_baisse, dt)
    moving_foot.add_segment(x, new_y, z_pied_leve, dt)
    com.append(np.array([-x, other_y]))
    t_base.append(dt)

    # on descend le pied
    other_foot.add_segment(-x, other_y, z_pied_baisse, dt)
    moving_foot.add_segment(x, new_y, z_pied_baisse, dt)
    com.append(np.array([-x, other_y]))
    t_base.append(dt)

    return new_y

y_left = 0
y_right = 0

y_left = do_step(posesLeft, posesRight, -0.3, y_left, y_right, step_dist/2)
for _ in range(5):
    y_right = do_step(posesRight, posesLeft, 0.3, y_right, y_left, step_dist)
    y_left = do_step(posesLeft, posesRight, -0.3, y_left, y_right, step_dist)
y_right = do_step(posesRight, posesLeft, 0.3, y_right, y_left, step_dist/2)

com = np.array(com).T

integral_dt = 0.02
t_base = np.arange(0, len(com[0])*dt-0.001, dt)
t_interp = np.arange(0, len(com[0])*dt-0.001, integral_dt)
com_interp = np.array([np.interp(t_interp, t_base, com[0]), np.interp(t_interp, t_base, com[1])])

def cost(xy):
    xy = np.reshape(xy, (-1,2)).T
    xy_interp = np.array([np.interp(t_interp, t_base, xy[0]), np.interp(t_interp, t_base, xy[1])])
    cp_list = cp(xy_interp, integral_dt, -z_pied_baisse)
    
    c = 0
    for i in range(len(com_interp)):
        c += np.sum(np.sum(np.square(cp_list[i] - com_interp[i]), axis=0) * integral_dt)
    return c

traj_com = fmin_bfgs(cost, com)
traj_com = np.reshape(traj_com, (-1,2))

for coord in traj_com:
    posesWaist.add_segment(coord[0], coord[1], 0, dt)


# calcul de toutes les positions articulaires avant affichage

increment = 0.1
dt = 1 / 60
nb_iter = int(increment / dt)

qs = [q_init]

for t in np.arange(0, posesLeft.max_t(), increment):
    ik = InverseKinematics(robot)
    ik.leftFootRefPose = SE3(eye(3), posesLeft.eval(t))
    ik.rightFootRefPose = SE3(eye(3), posesRight.eval(t))
    ik.waistRefPose = SE3(eye(3), posesWaist.eval(t))
    qs.append(ik.solve(qs[-1]))

# Affichage

while True:
    for i in range(0, len(qs)-1):
        q_diff = (qs[i+1] - qs[i]) / nb_iter
        q = qs[i]
        for j in range(nb_iter):
            q += q_diff
            robot.display(q)
            time.sleep(dt)
        robot.display(qs[i+1])
