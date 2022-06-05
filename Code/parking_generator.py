# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 19:27:14 2022

@author: misae
"""

import numpy as np

##############################################################################
# DIAGONAL
##############################################################################


def diagonal_empty(origin, angle, L, w, w_inter):
    x0 = origin[0]
    y0 = origin[1]

    x1 = x0 - L*np.sin(angle)
    y1 = y0 - L*np.cos(angle)

    x2 = x1 + w
    y2 = y1

    x3 = x0 + w
    y3 = y0

    x4 = x3 + w_inter
    y4 = y3

    return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]


def diagonal_full(origin, angle, L, w, w_inter):
    x0 = origin[0]
    y0 = origin[1]

    x1 = x0 + w + w_inter
    y1 = y0

    return [(x1, y1)]


def diagonal(origin, angle, L, w, w_inter, n):
    p0 = origin
    ps = [origin]
    for i in range(n):
        ps_aux = diagonal_empty(p0, angle, L, w, w_inter)
        for p in ps_aux:
            ps.append(p)
        p0 = ps_aux[-1]

    return ps


def d_polygon(origin, angle, L, w, w_inter, n, states):
    p0 = origin
    ps = [origin]
    for i in range(n):
        if states[i]:
            ps_aux = diagonal_full(p0, angle, L, w, w_inter)
        else:
            ps_aux = diagonal_empty(p0, angle, L, w, w_inter)
        for p in ps_aux:
            ps.append(p)
        p0 = ps_aux[-1]

    return ps

##############################################################################
# PARALLEL
##############################################################################


def parallel_empty(origin, L, w, w_inter):
    x0 = origin[0]
    y0 = origin[1]

    x1 = x0
    y1 = y0 - L

    x2 = x1 + w
    y2 = y1

    x3 = x0 + w
    y3 = y0

    x4 = x3 + w_inter
    y4 = y3

    return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]


def parallel_full(origin, L, w, w_inter):
    x0 = origin[0]
    y0 = origin[1]

    x1 = x0 + w + w_inter
    y1 = y0

    return [(x1, y1)]


def p_display(origin, L, w, w_inter, n):
    p0 = origin
    ps = [origin]
    for i in range(n):
        ps_aux = parallel_empty(p0, L, w, w_inter)
        for p in ps_aux:
            ps.append(p)
        p0 = ps_aux[-1]

    return ps


def p_polygon(origin, L, w, w_inter, n, states):
    p0 = origin
    ps = [origin]
    for i in range(n):
        if states[i]:
            ps_aux = parallel_full(p0, L, w, w_inter)
        else:
            ps_aux = parallel_empty(p0, L, w, w_inter)
        for p in ps_aux:
            ps.append(p)
        p0 = ps_aux[-1]

    return ps

##############################################################################
# TRACE
##############################################################################


def rectangle(origin, L, w):
    x = origin[0]
    y = origin[1]

    q1 = (x - L/2.0, y - w/2.0)
    q2 = (x + L/2.0, y - w/2.0)
    q3 = (x + L/2.0, y + w/2.0)
    q4 = (x - L/2.0, y + w/2.0)
    return [q1, q2, q3, q4]
