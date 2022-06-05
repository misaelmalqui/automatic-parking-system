# -*- coding: utf-8 -*-
"""
Created on Mon May  2 14:46:59 2022

@author: misae
"""

import numpy as np
from shapely.geometry import Polygon

##############################################################################
# CONSTANTS
##############################################################################

# meters to pixels
m2p = 3779.52/60.0

# colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLLOW = (232, 193, 28)
GREY = (200, 200, 200)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# manim colors
gray_e = (34, 34, 34)
red_e = (207, 80, 68)
yellow_e = (232, 193, 28)
gray_c = (136, 136, 136)
gray_d = (68, 68, 68)
green_e = (105, 156, 82)
green_c = (131, 193, 103)

# parking types
pk_types = {'diagonal': 0}

# dimensions
xmin = 0.0
xmax = 30.0
ymin = 0.0
ymax = 15.0

# running or not
running = True
parkingstate = False

# Program variables
dt = 0.10
ds = m2p*0.15
nmax = 300

# default parking configuration
pk_origin = (m2p*8.0, m2p*6.0)
pk_angle = np.pi/6.0*0.0
pk_L = m2p*5.0
pk_w = m2p*3.0
pk_w_inter = m2p*0.2
pk_n = 3
pk_states = [0, 0, 1, 0, 1, 0, 0]

# car dimensions
cwidth = 2.0
clength = 4.0
cstate = [m2p*2.0, m2p*8.0, m2p*0.0]

# parallel space
parallel_dx = m2p*2.0
parallel_dy = m2p*2.0
parallel_w = m2p*5.0
parallel_L = m2p*2.8

# trace
trace_w = m2p*0.15
trace_L = m2p*0.6
trace_d = m2p*0.15
trace_origin = [m2p*0.0, m2p*8.0]

##############################################################################
# INITIALIZATION
##############################################################################


def init(xmin, xmax, ymin, ymax, cwidth, clength, cstate):
    global XMIN, XMAX, YMIN, YMAX, corners, dims, bounds, cw, cl, q0
    XMIN = m2p*xmin
    XMAX = m2p*xmax
    YMIN = m2p*ymin
    YMAX = m2p*ymax

    corners = [(XMIN, YMIN), (XMIN, YMAX), (XMAX, YMAX), (XMAX, YMIN)]
    dims = (m2p*(ymax - ymin), m2p*(xmax - xmin))
    bounds = Polygon(corners)

    cw = cwidth
    cl = clength
    q0 = cstate


init(xmin, xmax, ymin, ymax, cwidth, clength,  cstate)
