# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 15:26:52 2022

@author: misae
"""

import pygame as pg
import math
import numpy as np
import os

from shapely.geometry import Point, Polygon
from shapely.ops import cascaded_union
import parking_generator as pkg
import parking_init as pki

# initialization
pg.init()

##############################################################################
# FUNCTIONS
##############################################################################


def m2p_conv(ps):
    ps_aux = []
    for i in range(len(ps)):
        ps_aux.append((pki.m2p*ps[i][0], pki.m2p*ps[i][1]))
    return ps_aux


def ray_track(x0, y0, angle, obstacles):
    x = x0
    y = y0

    finish = False

    while not finish:
        p = Point(x, y)
        for obstacle in obstacles:
            if p.within(obstacle.polygon):
                xf = x - np.cos(angle)*pki.ds
                yf = y - np.sin(angle)*pki.ds
                finish = True
                break

        if not p.within(pki.bounds):
            xf = x - np.cos(angle)*pki.ds
            yf = y - np.sin(angle)*pki.ds
            finish = True

        x = x + np.cos(angle)*pki.ds
        y = y + np.sin(angle)*pki.ds

    return (xf, yf)


def recognition_track(x0, y0, obstacles):
    angles = np.linspace(0, 2.0*np.pi, 420)
    psf = []
    for angle in angles:
        psf.append(ray_track(x0, y0, angle, obstacles))
    polf = Polygon(psf)
    return polf


def findcorners(state, width, length):
    x = state[0]
    y = state[1]
    angle = state[2]

    q1x = x + width/2.0*np.sin(angle)
    q1y = y - width/2.0*np.cos(angle)

    q2x = x + width/2.0*np.sin(angle) + length*np.cos(angle)
    q2y = y - width/2.0*np.cos(angle) + length*np.sin(angle)

    q3x = x - width/2.0*np.sin(angle) + length*np.cos(angle)
    q3y = y + width/2.0*np.cos(angle) + length*np.sin(angle)

    q4x = x - width/2.0*np.sin(angle)
    q4y = y + width/2.0*np.cos(angle)

    return [(q1x, q1y), (q2x, q2y), (q3x, q3y), (q4x, q4y)]


def findparallel(state):
    x = state[0]
    y = state[1]
    angle = state[2]

    p1x = x - pki.parallel_dx*np.cos(angle) + pki.parallel_dy*np.sin(angle)
    p1y = y - pki.parallel_dx*np.sin(angle) - pki.parallel_dy*np.cos(angle)

    p2x = x - pki.parallel_dx*np.cos(angle) + (pki.parallel_dy + pki.parallel_L)*np.sin(angle)
    p2y = y - pki.parallel_dx*np.sin(angle) - (pki.parallel_dy + pki.parallel_L)*np.cos(angle)

    p3x = x - (pki.parallel_dx + pki.parallel_w)*np.cos(angle) + (pki.parallel_dy + pki.parallel_L)*np.sin(angle)
    p3y = y - (pki.parallel_dx + pki.parallel_w)*np.sin(angle) - (pki.parallel_dy + pki.parallel_L)*np.cos(angle)

    p4x = x - (pki.parallel_dx + pki.parallel_w)*np.cos(angle) + pki.parallel_dy*np.sin(angle)
    p4y = y - (pki.parallel_dx + pki.parallel_w)*np.sin(angle) - pki.parallel_dy*np.cos(angle)

    return [(p1x, p1y), (p2x, p2y), (p3x, p3y), (p4x, p4y)]


def draw_trace(origin, screen):
    i = 0
    origin2 = [origin[0] + (pki.trace_L + pki.trace_d)*i, origin[1]]
    while origin2[0] < pki.XMAX:
        ps = pkg.rectangle(origin2, pki.trace_L, pki.trace_w)
        pg.draw.polygon(screen, pki.WHITE, ps)
        i = i + 1
        origin2 = [origin[0] + (pki.trace_L + pki.trace_d)*i, origin[1]]


##############################################################################
# OBJECTS
##############################################################################


class Env:
    def __init__(self, dimensions):
        # map dimensions
        self.height = dimensions[0]
        self.width = dimensions[1]

        # windows
        pg.display.set_caption('Third attempt')  # caption in the windows
        self.map = pg.display.set_mode((self.width, self.height), 8)

        self.font = pg.font.SysFont('Times New Roman', 20)
        self.text = self.font.render('default', True, pki.BLACK)
        self.textRect = self.text.get_rect()


class Obstacle:
    def __init__(self, positions):
        self.ps = positions
        self.polygon = Polygon(self.ps)

    def draw(self, screen):
        pg.draw.polygon(screen, pki.WHITE, self.ps)
        pg.draw.lines(screen, pki.BLACK, True, self.ps, width=3)


class P_diagonal:
    def __init__(self, origin, angle, L, w, w_inter, n, states):
        self.angle = angle
        self.origin = origin
        self.L = L
        self.w = w
        self.w_inter = w_inter
        self.n = n
        self.states = states

        # points for displaying
        y0 = self.origin[1]
        pts_total = [(pki.XMIN, y0)]
        pts_aux = pkg.diagonal(self.origin, self.angle, self.L,
                               self.w, self.w_inter, self.n)
        for p in pts_aux:
            pts_total.append(p)
        pts_total.append((pki.XMAX, y0))
        pts_total.append(pki.corners[3])
        pts_total.append(pki.corners[0])

        self.ps_display = pts_total

        # points for polygon
        y0 = self.origin[1]
        pts_total = [(pki.XMIN, y0)]
        pts_aux = pkg.d_polygon(self.origin, self.angle, self.L,
                                self.w, self.w_inter, self.n, self.states)
        for p in pts_aux:
            pts_total.append(p)
        pts_total.append((pki.XMAX, y0))
        pts_total.append(pki.corners[3])
        pts_total.append(pki.corners[0])

        self.ps_polygon = pts_total
        self.polygon = Polygon(pts_total)

    def draw_obstacle(self, screen):
        pg.draw.polygon(screen, pki.WHITE, self.ps_polygon)
        return None

    def draw_places(self, screen):
        pg.draw.lines(screen, pki.BLACK, True, self.ps_display, width=3)
        return None

    def inside_ind(self, x, y):
        return Point(x, y).within(self.polygon)

    def inside(self, x, y):
        vfunc = np.vectorize(self.inside_ind)
        return vfunc(x, y)


class P_parallel:
    def __init__(self, origin, n, states):
        self.origin = origin
        self.L = pki.m2p*3.5
        self.w = pki.m2p*6.5
        self.w_inter = pki.m2p*0.2
        self.n = n
        self.states = states

        # points for displaying
        y0 = self.origin[1]
        pts_total = [(pki.XMIN, y0)]
        pts_aux = pkg.p_display(self.origin, self.L,
                                self.w, self.w_inter, self.n)
        for p in pts_aux:
            pts_total.append(p)
        pts_total.append((pki.XMAX, y0))
        pts_total.append(pki.corners[3])
        pts_total.append(pki.corners[0])

        self.ps_display = pts_total

        # points for polygon
        y0 = self.origin[1]
        pts_total = [(pki.XMIN, y0)]
        pts_aux = pkg.p_polygon(self.origin, self.L,
                                self.w, self.w_inter, self.n, self.states)
        for p in pts_aux:
            pts_total.append(p)
        pts_total.append((pki.XMAX, y0))
        pts_total.append(pki.corners[3])
        pts_total.append(pki.corners[0])

        self.ps_polygon = pts_total
        self.polygon = Polygon(pts_total)

    def draw_obstacle(self, screen):
        pg.draw.polygon(screen, pki.gray_d, self.ps_polygon)
        return None

    def draw_places(self, screen):
        pg.draw.lines(screen, pki.WHITE, True, self.ps_display, width=3)
        return None

    def inside_ind(self, x, y):
        return Point(x, y).within(self.polygon)

    def inside(self, x, y):
        vfunc = np.vectorize(self.inside_ind)
        return vfunc(x, y)


class Car:
    def __init__(self, state, width, length):
        # robot dims
        self.w = pki.m2p*width
        self.L = pki.m2p*length
        # initial state
        self.state = state
        self.v = 0.0
        self.phi = 0.0
        self.maxspeed = pki.m2p*0.5
        self.maxphi = np.pi/6.0
        self.environment = Polygon(findcorners(state, width, length))

        # parking spaces
        self.parallel = findparallel(self.state)
        # parking maneuver counter
        self.counter = 0

    def recognize(self, obstacles):
        pol_aux = recognition_track(self.state[0], self.state[1], obstacles)
        self.environment = cascaded_union([pol_aux, self.environment])
        return

    def draw_availability(self, screen):
        xs, ys = car.environment.exterior.xy
        pg.draw.polygon(environment.map, pki.red_e,
                        [(xs[i], ys[i]) for i in range(len(xs))], width=3)
        return

    def draw(self, screen):
        # pg.draw.circle(environment.map, pki.BLACK,
        #                (car.state[0], car.state[1]), 5)
        pg.draw.lines(screen, pki.yellow_e, True, findcorners(self.state, self.w, self.L), width=3)

    def draw_parallel(self, screen):
        pg.draw.lines(screen, pki.green_e, True, self.parallel, width=4)

    def move(self):
        # input
        pressed = pg.key.get_pressed()
        if pressed[pg.K_UP]:
            self.v = self.v + pki.m2p*0.03
        if pressed[pg.K_DOWN]:
            self.v = self.v - pki.m2p*0.03
        if pressed[pg.K_RIGHT]:
            self.phi = self.phi + np.pi/6.0*pki.dt
        elif pressed[pg.K_LEFT]:
            self.phi = self.phi - np.pi/6.0*pki.dt
        else:
            self.phi = 0.0

        # dynamics
        self.state[0] += self.v*np.cos(self.state[2])*pki.dt
        self.state[1] += self.v*np.sin(self.state[2])*pki.dt
        self.state[2] += self.v/self.L*np.tan(self.phi)*pki.dt

        # self.rotated = pygame.transform.rotozoom(self.img, -math.degrees(self.theta), 1)
        # self.rect = self.rotated.get_rect(center=(self.x, self.y))

        # limits motion
        self.v = np.min([self.v, self.maxspeed])
        self.v = np.max([self.v, -self.maxspeed])
        self.phi = np.min([self.phi, self.maxphi])
        self.phi = np.max([self.phi, -self.maxphi])

        # update parallel spaces
        self.parallel = findparallel(self.state)

    def m_parallel(self):
        if self.counter <= 80:
            self.v = -self.maxspeed
            self.phi = -np.pi/4.2
        elif self.counter <= 161:
            self.v = -self.maxspeed
            self.phi = np.pi/4.2
        else:
            self.v = 0.0
            self.phi = 0.0
        self.state[0] += self.v*np.cos(self.state[2])*pki.dt
        self.state[1] += self.v*np.sin(self.state[2])*pki.dt
        self.state[2] += self.v/self.L*np.tan(self.phi)*pki.dt
        self.counter = self.counter + 1

#    def battery_place(self):
#        q1 = (self.state)


##############################################################################
# SET UP
##############################################################################

environment = Env(pki.dims)

car = Car(pki.q0, pki.cw, pki.cl)

# parking = P_diagonal(pki.pk_origin, pki.pk_angle, pki.pk_L,
#                      pki.pk_w, pki.pk_w_inter, pki.pk_n, pki.pk_states)
parking = P_parallel(pki.pk_origin, pki.pk_n, pki.pk_states)

##############################################################################
# RUNNING
##############################################################################

while pki.running:
    # Exit condition
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pki.running = False

    environment.map.fill(pki.gray_e)
    # for ob in obs:
    #    ob.draw(environment.map)

    # UPDATE
    # car.state[0] = car.state[0] + 5
    if not car.environment.contains(Polygon(car.parallel)):
        car.move()
    else:
        car.m_parallel()
        car.draw_parallel(environment.map)

    # DRAWING THE CAR AND PARKING
    parking.draw_obstacle(environment.map)
    parking.draw_places(environment.map)
    draw_trace(pki.trace_origin, environment.map)

    car.recognize([parking])
    # car.draw_availability(environment.map)

    car.draw(environment.map)

    # DISPLAY
    pg.display.update()

pg.quit()
