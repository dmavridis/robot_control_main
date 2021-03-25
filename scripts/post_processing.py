#! /usr/bin/env python

import numpy as numpy
import math
import matplotlib.pyplot as plt

def draw_points(points):
    """
    Points are in the format (x,y, theta)
    """
    #plt.figure(figsize=(5,5))

    ax = plt.axes()
    ax.set_xlim(-1.05,1.05)
    ax.set_ylim(-1.05,1.05)
    for p in points:
        x, y, dx, dy = point_to_arrow(p)
        ax.arrow(x, y, dx, dy, head_width = 0.04, fc='red', linewidth=0.1)
    plt.grid()
    plt.show()

def point_to_arrow(point):
    arrow_len = 0.2
    x = point.x
    y = point.y
    theta = point.z
    dx = arrow_len*math.cos(theta)
    dy = arrow_len*math.sin(theta)
    return x, y, dx, dy

def point_to_arrow2(point):
    arrow_len = 0.2
    x = point[0]
    y = point[1]
    theta = point[2]
    dx = arrow_len*math.cos(theta)
    dy = arrow_len*math.sin(theta)
    return x, y, dx, dy