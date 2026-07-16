#!/usr/bin/python3

import typing

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.lines import Line2D

from Dubins import BasicPath,DubinsMove,poses_XY_dist
from plotting import plot_BasicPath_obstacle


from scipy.optimize import direct,OptimizeResult

def done(ax:Axes):
    ax.set_aspect('equal')
    plt.show()
    exit(0)
    
def min_path_dist(p:BasicPath,pbis:BasicPath):
    def opt_fun(v):
        l1 = p.pose_at(v[0])
        l2 = pbis.pose_at(v[1])
        return poses_XY_dist(l1,l2)
    return direct(opt_fun,[(0,p.duration()),(0,pbis.duration())])

def min_straight_lstsq(p:BasicPath,pbis:BasicPath):
    assert p.type is DubinsMove.STRAIGHT
    assert pbis.type is DubinsMove.STRAIGHT
    
    s1 = np.array(p.start().astuple()[:2])
    s2 = np.array(pbis.start().astuple()[:2])
    
    b = s1 - s2
    A = np.array(
        [
            [-p.p1,pbis.p1],
            [-p.p2,pbis.p2]
        ]
    )
    print(A,b)
    
    return np.linalg.lstsq(A,b)

fig,ax = plt.subplots()

o = BasicPath(
    DubinsMove.STRAIGHT,
    2000,
    1050,
    -1000,
    0,
    0,1,0,0
)

plot_BasicPath_obstacle(ax,o)

p1 = BasicPath(
    DubinsMove.STRAIGHT,
    932.9630739080485,
    1079.5270249176237,
    -44.852813742385734,
    0,
    1,
    0,
    0,
    0
)
plot_BasicPath_obstacle(ax,p1)[0].set_color('b')

print(min_path_dist(p1,o))
# print(min_straight_lstsq(p1,o))

done(ax)

p2 = BasicPath(
    DubinsMove.STRAIGHT,
    71.080377549109386,
    22.153456264911696,
    -123.77352214838169,
    0,
    -0.14830263340974387,
    -0.98894202505694706,
    0,0
)
plot_BasicPath_obstacle(ax,p2)[0].set_color('b')

p3 = BasicPath(
    DubinsMove.LEFT,
    35.462210771003342,
    -11.516243797058127,
    -560,
    0,
    40,
    0.025,
    0,
    3.8258337111096061
)
plot_BasicPath_obstacle(ax,p3)[0].set_color('b')

p4 = BasicPath(
    DubinsMove.STRAIGHT,
    1011.5162437970581,
    -11.51624379705811,
    -600,
    0,
    1,
    0,
    0,
    0
)
plot_BasicPath_obstacle(ax,p4)[0].set_color('b')

p5 = BasicPath(
    DubinsMove.STRAIGHT,
    505.75812189852905,
    494.24187810147095,
    -600,
    0,
    1,
    0,
    0,
    0
)
plot_BasicPath_obstacle(ax,p5)[0].set_color('b')