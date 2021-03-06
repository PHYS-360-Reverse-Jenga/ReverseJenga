# -*- coding: utf-8 -*-
"""
Created on Fri Feb 10 15:10:53 2017

@author: sinkovitsd
"""

import pygame
from vec2d import Vec2d
import random
import math

RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
CYAN = (0,255,255)
MAGENTA = (255,0,255)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)

class Shape:
    def __init__(self, pos, vel, angle, angvel, color, mass, moment, points):
        self.pos = Vec2d(pos)
        self.vel = Vec2d(vel)
        self.force = Vec2d(0,0)
        self.color = color
        self.origpoints = []
        self.points = []
        for p in points:
            self.origpoints.append(Vec2d(p))
            self.points.append(Vec2d(p))
        if not self.is_convex():
            raise ValueError('Initiated polygon is not convex!')
        self.create_origaxes()
        self.angle = angle
        self.angvel = float(angvel)
        self.torque = 0.0
        self.mass = mass
        self.massinv = 1.0/mass
        self.moment = moment
        self.momentinv = 1.0/moment
        self.visible = True
        self.sleeping = False
        self.update_points()
        self.update_axes()    
        
    def create_origaxes(self):
        self.origaxes = []
        self.axes = []
        p0 = self.origpoints[-1]
        for p in self.origpoints:
            axis = (p - p0).perpendicular_normal()
            p0 = p
            """ Test for uniqueness:
                If two axes are parallel, only one is needed. """
            unique = True
            for a in self.origaxes:
                if abs(axis.dot(a)) == 1:
                    unique = False
                    break
            if unique:
                self.origaxes.append(axis)
                self.axes.append(axis)
       
    def update_points(self):
        """ (3) Replace self.origpoints[i] with a calculation of the current 
                location of the point based on pos and angle. """
        cos_angle = math.cos(self.angle)
        sin_angle = math.sin(self.angle)
        for i in range(len(self.origpoints)):
            rotated = Vec2d(
              self.origpoints[i].x*cos_angle - self.origpoints[i].y*sin_angle,\
              self.origpoints[i].y*cos_angle + self.origpoints[i].x*sin_angle)
            self.points[i] = self.pos + rotated
            
    def update_axes(self):
        """ (4) Replace self.origaxes[i] with a calculation of the current 
                axis direction based on the current angle. """
        cos_angle = math.cos(self.angle)
        sin_angle = math.sin(self.angle)
        for i in range(len(self.origaxes)):
            rotated = Vec2d(
              self.origaxes[i].x*cos_angle - self.origaxes[i].y*sin_angle,\
              self.origaxes[i].y*cos_angle + self.origaxes[i].x*sin_angle)
            self.axes[i] = rotated
                    
    def add_impulse(self, imp, pos):
        """ (5) change vel and angvel based on added impulse """
        #print(self.color, imp, pos)
        self.vel    += imp*self.massinv#Vec2d(0,0)
        self.angvel += (pos - self.pos).cross(imp)*self.momentinv#0.0
        #print(self.vel, self.angvel)
        
    def is_convex(self): # Need to do this later
        return True
        
    def draw(self, screen):
        if self.visible:
            self.update_points()
            n = len(self.points)
            #if self.color == (255, 0, 0):
            #    print(self.color, self.vel.length, abs(self.angvel), self.sleeping)
            if n > 2:
                pygame.draw.polygon(screen, self.color, self.points, 0)       
            else:
                pygame.draw.line(screen, self.color, self.points[0], self.points[-1], 1)
            
class World:
    def __init__(self, width_pixels, height_pixels, bg_color = (255,255,255)):
        pygame.init()
        self.width = width_pixels
        self.height = height_pixels
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.bg_screen = pygame.Surface((self.width, self.height))
        self.bg_screen.fill(bg_color)
        self.shapes = []

    def add(self, shape):
        self.shapes.append(shape)
        
    def append(self, shape):
        self.shapes.append(shape)
        
    def prepend(self, shape):
        self.shapes.insert(0, shape)
        
    def remove(self, shape):
        self.shapes.remove(shape)
        
    def display(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)
        pygame.display.flip()

    def display_noflip(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)

def update_pos(shapes, dt):
    """ (1) add angle update """
    for p in shapes:
        if not p.sleeping:
            p.pos   += p.vel*dt
            p.angle += p.angvel*dt

def update_vel(shapes, dt):
    """ (2) add angvel update """
    for p in shapes:
        if not p.sleeping:
            p.vel    += p.force*p.massinv*dt
            p.angvel += p.torque*p.massinv*dt
        
def update_force(shapes, world):
    for p in shapes:
        p.force = Vec2d(0,0)
        p.torque = 0.0
    for p in shapes:
        gravity = Vec2d(0, 0.05)
        p.force += p.mass * gravity
    
class Wall:
    def __init__(self, normal, position, e=1, mu=0):
        self.normal = normal.normalized()
        self.pos = position
        self.e = e
        self.mu = mu
        
def collide(shape1, shape2):
    dmin = 0
    disp = Vec2d(0,0)
    j = 0
    axes = shape1.axes + shape2.axes
    for j,axis in enumerate(axes):
        p1min = axis.dot(shape1.points[0])
        p1max = p1min
        i1min = 0
        i1max = 0
        for i in range(1,len(shape1.points)):
            p = axis.dot(shape1.points[i])
            if p > p1max:
                p1max = p
                i1max = i
            elif p < p1min:
                p1min = p
                i1min = i

        p2min = axis.dot(shape2.points[0])
        p2max = p2min
        i2min = 0
        i2max = 0
        for i in range(1, len(shape2.points)):
            p = axis.dot(shape2.points[i])
            if p > p2max:
                p2max = p
                i2max = i
            elif p < p2min:
                p2min = p
                i2min = i

        d1 = p1max - p2min
        d2 = p2max - p1min
        if d1 <= 0 or d2 <= 0:
            return (False, None, None, None, None)
        else:
            if d1 < d2:
                axis *= -1
                d = d1
                if j < len(shape1.axes): # axis belongs to shape1
                    pt = shape2.points[i2min]
                    vec1 = pt - d*axis - shape1.pos
                    vec2 = pt - shape2.pos
                else:
                    pt = shape1.points[i1max]
                    vec1 = pt - shape1.pos
                    vec2 = pt + d*axis - shape2.pos
            else:
                d = d2
                if j < len(shape1.axes):
                    pt = shape2.points[i2max]
                    vec1 = pt - d*axis - shape1.pos
                    vec2 = pt - shape2.pos
                else:
                    pt = shape1.points[i1min]
                    vec1 = pt - shape1.pos
                    vec2 = pt + d*axis - shape2.pos
            if dmin == 0 or d < dmin:
                dmin = d
                disp = d*axis
                point = pt
                vector1 = vec1
                vector2 = vec2
    return (True, disp, point, vector1, vector2)

def handle_collisions(shapes, world, dt):
    DEG = math.pi/180
    walls = [Wall(Vec2d(0,-1), Vec2d(0,world.height), 0.7, 0.3),
             Wall(Vec2d(1,0), Vec2d(0,0), 0.7, 0.1),
             Wall(Vec2d(-1,0), Vec2d(world.width, 0), 0.7, 0.1)]
    e = 0.7
    mu = 0.3
    collided = False
    sleep_vel = 0.005
    sleep_angvel = 0.001
    for p in shapes:
        p.update_points()
        p.update_axes()
    for i in range(len(shapes)):
        p = shapes[i]
        for wall in walls:
            max_d = -1e15
            point = Vec2d(0,0)
            for t in p.points:
                d = wall.normal.dot(wall.pos - t)
                if d > max_d:
                    max_d = d
                    point = t
            if max_d > 0:
                p.sleeping = False
                r = point - p.pos
                collided = True
                vrel = p.vel + p.angvel*r.perpendicular()
                vrel_normal = vrel.dot(wall.normal)
                if vrel_normal < 0:
                    v_target_normal = -(1 + wall.e)*vrel_normal
                    parallel = wall.normal.perpendicular()
                    v_target_parallel = -vrel.dot(parallel)
                    r_normal = r.dot(wall.normal)
                    r_parallel = r.dot(parallel)
                    minv_parallel = p.massinv + r_parallel*r_parallel*p.momentinv
                    minv_normal = p.massinv + r_normal*r_normal*p.momentinv
                    minv_both = r_normal*r_parallel*p.momentinv
                    m = 1.0/(minv_parallel*minv_normal - minv_both*minv_both)
                    imp_normal = (minv_normal*v_target_normal 
                                      + minv_both*v_target_parallel)*m
                    imp_parallel = (minv_parallel*v_target_parallel
                                        + minv_both*v_target_normal)*m
                    if imp_parallel < 0:
                        imp_parallel *= -1
                        parallel *= -1
                        minv_both *= -1
                        v_target_parallel *= -1
                    if imp_parallel > wall.mu*imp_normal:
                        imp_normal = v_target_normal/(minv_parallel - wall.mu*minv_both)
                        imp_parallel = wall.mu*imp_normal
                    imp = imp_normal*wall.normal + imp_parallel*parallel
                    p.add_impulse(imp, point)
                rxn = r.cross(wall.normal)
                mm = 1.0/(p.massinv + rxn*rxn*p.momentinv)
                p.pos += wall.normal*max_d*p.massinv*mm
                p.angle += max_d*rxn*p.momentinv*mm
                p.update_points()
                p.update_axes()
                if p.vel.length < sleep_vel and abs(p.angvel) < sleep_angvel:
                    p.sleeping = True
        for j in range(i):#range(len(shapes)):
            if i==j: continue
            q = shapes[j]
            (collision, disp, point, pr, qr) = collide(p, q)
            if collision:
                collided = True
                p.sleeping = False
                q.sleeping = False
                normal = disp.normalized()
                vrel = (p.vel + p.angvel*pr.perpendicular()
                        -(q.vel + q.angvel*qr.perpendicular()))
                vrel_normal = vrel.dot(normal)
                if vrel_normal < 0:
                    v_target_normal = -(1 + e)*vrel_normal
                    parallel = normal.perpendicular()
                    v_target_parallel = -vrel.dot(parallel)
                    pr_normal = pr.dot(normal)
                    pr_parallel = pr.dot(parallel)
                    qr_normal = qr.dot(normal)
                    qr_parallel = qr.dot(parallel)
                    minv_parallel = (p.massinv + pr_parallel*pr_parallel*p.momentinv
                                     + q.massinv + qr_parallel*qr_parallel*q.momentinv)
                    minv_normal = (p.massinv + pr_normal*pr_normal*p.momentinv
                                   + q.massinv + qr_normal*qr_normal*q.momentinv)
                    minv_both = (pr_normal*pr_parallel*p.momentinv 
                                 + qr_normal*qr_parallel*q.momentinv)
                    m = 1.0/(minv_parallel*minv_normal - minv_both*minv_both)
                    imp_normal = (minv_normal*v_target_normal 
                                      + minv_both*v_target_parallel)*m
                    imp_parallel = (minv_parallel*v_target_parallel
                                        + minv_both*v_target_normal)*m
                    if imp_parallel < 0:
                        imp_parallel *= -1
                        parallel *= -1
                        minv_both *= -1
                    if imp_parallel > mu*imp_normal:
                        imp_normal = v_target_normal/(minv_parallel - mu*minv_both)
                        imp_parallel = mu*imp_normal
                    imp = imp_normal*normal + imp_parallel*parallel
                    p.add_impulse(imp, point)
                    q.add_impulse(-imp, point)
                prxn = pr.cross(normal)
                qrxn = qr.cross(normal)
                mm = 1.0/(p.massinv + q.massinv 
                        + prxn*prxn*p.momentinv + qrxn*qrxn*q.momentinv)
                p.pos += disp*p.massinv*mm
                q.pos -= disp*q.massinv*mm
                d = disp.length
                p.angle += d*prxn*p.momentinv*mm
                q.angle -= d*qrxn*q.momentinv*mm
                p.update_points()
                q.update_points()
                p.update_axes()
                q.update_axes()
                if p.vel.length < sleep_vel and abs(p.angvel) < sleep_angvel:
                    p.sleeping = True
                if q.vel.length < sleep_vel and abs(q.angvel) < sleep_angvel:
                    q.sleeping = True
    return collided

class Rectangle(Shape):
    def __init__(self, pos, vel, angle, angvel, color, density, length, height):
        self.unscaled_points = [Vec2d(0.5, 0.5), Vec2d(-0.5, 0.5), 
                                Vec2d(-0.5, -0.5), Vec2d(0.5, -0.5)]
        points = []
        for p in self.unscaled_points:
            points.append(Vec2d(p.x*length, p.y*height))
        mass = density*length*height
        moment = (mass/12)*(length*length + height*height)
        super().__init__(pos, vel, angle, angvel, color, mass, moment, points)
           
def random_color(minimum, maximum):
    while True:
        color = (random.randrange(256), random.randrange(256), random.randrange(256))
        value = sum(color)
        if value > minimum and value < maximum:
            break
    return color
    
def main():
    pygame.init()
    world = World(800, 600, WHITE)
    world.display()
    DEG = math.pi/180
    moving = []

    clock = pygame.time.Clock()
    done = False
    density = 1 # mass / area
    timesteps = 0
    """
    shape1 = Rectangle((400,400), (0,0), 0, 0, BLUE, 1, 200, 50)
    world.add(shape1)
    shape1.add_impulse(Vec2d(0,-5000), Vec2d(500,400))
    moving.append(shape1)
    shape2 = Rectangle((410,200), (0,0), 45*DEG, 0.01, GREEN, 1, 200, 50)
    world.add(shape2)
    shape2.add_impulse(Vec2d(0,-5000), Vec2d(500,400))
    moving.append(shape2)
    shape3 = Rectangle((390,000), (0,0), -45*DEG,-0.0, RED, 1, 200, 50)
    world.add(shape3)
    #shape3.add_impulse(Vec2d(0,-5000), Vec2d(500,400))
    moving.append(shape3)
    """
    """
    shape = Rectangle((400,400), (0,0), 70*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    """
    
    shape = Rectangle((400,550), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    shape = Rectangle((400,450), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    shape = Rectangle((400,350), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    shape = Rectangle((400,250), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    shape = Rectangle((400,150), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    shape = Rectangle((400,50), (0,0), 0*DEG, 0.0, RED, 1, 200, 50)
    world.add(shape)
    moving.append(shape)
    
    while not done:
        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT: # Close window clicked
                done = True
                break

        # Velocity Verlet method
        n = 1
        dt = 1 / n
        collide_max = 10
        for i in range(n):        
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            update_pos(moving, dt)
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            collide_count = 0
            collide_dt = dt
            while (collide_count < collide_max 
                   and handle_collisions(moving, world, collide_dt)):
                collide_count += 1
                collide_dt = 0
            #print(collide_count)
        #print(shape.pos, shape.angle)
        world.display()
        clock.tick(60)
        
    pygame.quit() # quit nicely, so the program window doesn't hang

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        pygame.quit()
        raise e
           