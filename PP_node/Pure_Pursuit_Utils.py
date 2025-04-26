import os
import sys
import math
import numpy as np
import json

class C:
    # PID config
    Kp = 0.3  # proportional gain

    # system config
    Ld = 3  # look ahead distance
    kf = 0.1  # look forward gain
    dt = 0.1  # T step
    dist_stop = 0.1  # stop distance
    dc = 0.0

    # vehicle config
    RF = 2.7  # [m] distance from rear to vehicle front end of vehicle
    RB = 1.8  # [m] distance from rear to vehicle back end of vehicle
    W = 2.4  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 2.5  # [m] Wheel base
    TR = 0.44  # [m] Tyre radius
    TW = 0.7  # [m] Tyre width
    MAX_STEER = 0.30
    MAX_ACCELERATION = 0.5
    GearRatio = 16.18 # gear ratio from steering wheel to front wheel 

class Node:
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        # delta = self.limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.direct = direct
        self.v += self.direct * a * C.dt

    @staticmethod
    def limit_input(delta):
        if delta > 1.2 * C.MAX_STEER:
            return 1.2 * C.MAX_STEER

        if delta < -1.2 * C.MAX_STEER:
            return -1.2 * C.MAX_STEER

        return delta

class PATH:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        # 输入整个路径
        self.ind_end = len(self.cx) - 1
        self.index_old = None

    def target_index(self, node):
        """
        search index of target point in the reference path.
        the distance between target point and current position is ld
        :param node: current information
        :return: index of target point
        """

        if self.index_old is None:
            self.calc_nearest_ind(node)

        Lf = C.kf * node.v + C.Ld

        for ind in range(self.index_old, self.ind_end + 1):
            if self.calc_distance(node, ind) > Lf:
                self.index_old = ind
                return ind, Lf

        self.index_old = self.ind_end

        return self.ind_end, Lf

    def calc_nearest_ind(self, node):
        """
        calc index of the nearest point to current position
        :param node: current information
        :return: index of nearest point
        """

        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        self.index_old = ind

    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])
    
def pure_pursuit(node, ref_path, index_old):
    """
    pure pursuit controller
    :param node: current information
    :param ref_path: reference path: x, y, yaw, curvature
    :param index_old: target index of last time
    :return: optimal steering angle
    """

    ind, Lf = ref_path.target_index(node)  # target point and pursuit distance
    ind = max(ind, index_old)

    tx = ref_path.cx[ind]
    ty = ref_path.cy[ind]

    alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
    delta = math.atan2(2.0 * C.WB * math.sin(alpha), Lf)

    return delta, ind

def pid_control(target_v, v, dist, direct):
    """
    PID controller and design speed profile.
    :param target_v: target speed (forward and backward are different)
    :param v: current speed
    :param dist: distance from current position to end position
    :param direct: current direction
    :return: desired acceleration
    """

    a = 0.3 * (target_v - v)

    if dist < 2.0:
        if (v > 2) and (direct > 0):
            a = -0.5
        elif (v > 1) and (direct < 0):
            a = -0.3

    if a > C.MAX_ACCELERATION:
        a = C.MAX_ACCELERATION
    
    if direct < 0:
        a = 0.5*a
        
    return a

if __name__ == '__main__':
    pass