# -*- coding: utf-8 -*-
"""
Created on Mon May  2 14:46:01 2022

@author: misae
"""

# DEPENDENCIES

import numpy as np
from shapely.geometry import Point, Polygon
import parking_init as pki

def ray_track(x0, y0, angle, obstacles):
    x = x0
    y = y0

    finish = False

    while not finish:
        p = Point(x, y)
        for obstacle in obstacles:
            if p.within(obstacle.polygon):
                xf = x - np.cos(angle)*ds
                yf = y - np.sin(angle)*ds
                finish = True
                break

        if not p.within(bounds):
            xf = x - np.cos(angle)*ds
            yf = y - np.sin(angle)*ds
            finish = True

        x = x + np.cos(angle)*ds
        y = y + np.sin(angle)*ds

    return (xf, yf)


def recognition_track(x0, y0, obstacles):
    angles = np.linspace(0, 2.0*np.pi, 400)
    psf = []
    for angle in angles:
        psf.append(ray_track(x0, y0, angle, obstacles))
    obsf = obstacle(psf)
    return obsf

ob = pki
print(ray_track(0.0, 0.0, 0.0, [Polygon([(1, 1), (4, 4), (7, 0)])]))