# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 15:52:16 2017

@author: Leslie Murphy, Patrick Marquard, Ian Gorman
"""
import pygame
from vec2d import Vec2d
import random
import math
from WorldClass import *
from ShapeClass import *
from ParticleClass import *

RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
CYAN = (0,255,255)
MAGENTA = (255,0,255)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)

def update_pos(particles, dt):
    for p in particles:
        p.pos += p.vel*dt

def update_vel(particles, dt):
    for p in particles:
        p.vel += p.acc*dt
        
def update_acc(particles, world):
    set_acc(particles, Vec2d(0,0))
    gravity = Vec2d(0,0.5)
    add_acc(particles, gravity)
    kw = 1e4
    bw = 0#0.01*kw
    G = 0
    k = 1e4
    wall_data = [[0,0,0]] # no walls
    #wall_data = [[0,0,-1], [0, world.width, 1], [1, world.height, 1], [1,0,-1]] # all four walls
    # [(0=x; 1=y), (location), (1=repel too large; -1=repel too small; 0=inactive)]
    for p in particles:
        force = Vec2d(0, 0)
        for q in particles:
            if p != q:
                rvec = p.pos - q.pos
                rdir = rvec.normalized()
                rsqrd = rvec.get_length_sqrd()
                gforce = -G*p.mass*q.mass/rsqrd*rdir # Gravity
                R = p.radius + q.radius
                Rsqrd = R*R
                if rsqrd > Rsqrd:
                    force += gforce
        p.acc += p.massinv*force

def handle_collisions(particles, world):
    #wall_data = [[0,0,0]] # no walls
    e = 0.3
    ew = 0.3
    wall_data = [[0,0,-1], [0, world.width, 1], [1, world.height, 1], [1,0,-1]] # all four walls
    # [(0=x; 1=y), (location), (1=repel too large; -1=repel too small; 0=inactive)]
    for p in particles:
        for w in wall_data:
            if w[2] != 0:
                d = (p.pos[w[0]] - w[1])*w[2] + p.radius
                if d > 0:
                    rdir = Vec2d(0,0)
                    rdir[w[0]] = -w[2] # unit vector from wall toward particle
                    J = -(1+ew)*p.vel.dot(rdir)
                    if J > 0:
                        p.vel += J*rdir
                    #p.pos -= d
                    p.pos += rdir*d
                
                
        for q in particles:
            if p != q:
                rvec = p.pos - q.pos
                rsqrd = rvec.get_length_sqrd()
                R = p.radius + q.radius
                
                
                Rsqrd = R*R
                if rsqrd < Rsqrd:
                    rdir = rvec.normalized()
                    d = R - rvec.get_length()
                    J_div_r = -(1+e)*rvec.dot(p.vel - q.vel)/(rsqrd*(p.massinv + q.massinv))
                    if J_div_r > 0:
                        p.vel += J_div_r*rvec*p.massinv
                        q.vel -= J_div_r*rvec*q.massinv
                    
                    p.pos += rdir * d/2
                    q.pos -= rdir * d/2
                    
                    #attempt at change with porporinal mass
                    #p.pos += rdir * d * p.massinv / (p.massiv + q.massinv)
                    #q.pos -= rdir*d*p.massinv/(p.massiv+q.massinv)
    
    
                        
def add_acc(particles, accel):
    for p in particles:
        p.acc += accel
        
def set_acc(particles, accel):
    for p in particles:
        p.acc = Vec2d(accel) 
        # Vec2d constructor necessary to make a copy of accel,
        # otherwise, adding to one particles self.acc adds to all the others at once!!! 

def ellipse_surface(width, height, color = (0,0,0)):
    image = pygame.Surface((width, height))
    transparent = (255 - color[0], 255 - color[1], 255 - color[2])
    image.fill(transparent)
    image.set_colorkey(transparent)
    pygame.draw.ellipse(image, color, pygame.Rect((0,0), (width, height)))
    return image

def rect_surface(width, height, color = (0,0,0)):
    image = pygame.Surface((width, height))
    image.fill(color)
    return image

    
def main():
    pygame.init()
    world = World(800, 600, WHITE)
    world.display()
    
    moving = []

    clock = pygame.time.Clock()
    done = False
    new_particle_needed = True
    launching = False
    growth_rate = 10 # area / millisecond
    density = 1 # mass / area
    particle = None
    color = BLACK
    invsqrtpi = 1.0/math.sqrt(math.pi)
    timesteps = 0
    move_time = 0
    draw_time = 0
    wait_time = 0
    event_time = 0
    timing = False
    old_time = pygame.time.get_ticks()
    grow_time = old_time
    diameter = 40
    area = math.pi*(diameter/2)**2
    width = 80
    height = 20
    #default shape
    shape = ellipse_surface(diameter,diameter, BLACK)

    while not done:
        time = pygame.time.get_ticks()
        if timing: timesteps += 1
        if timing: wait_time += time - old_time
        old_time = time
        
        key = pygame.key.get_pressed()
        
        #change shape spawned
        if key[pygame.K_c]:
            shape = ellipse_surface(diameter,diameter, color)
        if key[pygame.K_r]:
            shape = rect_surface(width,height, color)

        # Create a new particle to replace one that was launched
        if new_particle_needed:
            new_particle_needed = False
            color = (random.randint(0,191), random.randint(0,191), random.randint(0,191))
            #clikc adds circle
            particle = Particle(Vec2d(pygame.mouse.get_pos()), (0,0), 1, shape)# ellipse_surface(diameter, diameter, color))
            radius = math.sqrt(area)*invsqrtpi
            diameter = 2*int(radius)
            particle.radius = radius
            particle.mass = density*area
            particle.massinv = 1.0/particle.mass
            world.add(particle)
            launching = False
            
        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT: # Close window clicked
                done = True
                break
            if event.type == pygame.KEYDOWN:
                timing = True
            # Release particle with initial velocity
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1: # left mouse button
                    particle.vel = 4*(event.pos - particle.pos)/particle.radius
                    particle.image.set_alpha(255)
                    moving.append(particle)
                    new_particle_needed = True
        
        # New particle follows pointer until mouse is pressed
        if not launching:
            pos = Vec2d(pygame.mouse.get_pos())
            particle.visible = (pygame.mouse.get_focused() and pos.x > 0 and pos.y > 0
                                and pos.x < world.width-1 and pos.y < world.height-1)
            particle.pos = pos
        
        time = pygame.time.get_ticks()
        if timing: event_time += time - old_time
        old_time = time
        
        # Velocity Verlet method
        n = 1
        dt = 1 / n
        for i in range(n):        
            update_acc(moving, world)
            update_vel(moving, 0.5*dt)
            update_pos(moving, dt)
            update_acc(moving, world)
            update_vel(moving, 0.5*dt)
            handle_collisions(moving, world)

        time = pygame.time.get_ticks()
        if timing: move_time += time - old_time
        old_time = time
        
        world.display()                
        
        time = pygame.time.get_ticks()
        if timing: draw_time += time - old_time
        old_time = time
        
        clock.tick(60) # wait so that this only updates 60 fps maximum
        
    if timing:
        print("Move time  =", move_time / timesteps, "ms")
        print("Draw time  =", draw_time / timesteps, "ms")
        print("Event time =", event_time / timesteps, "ms")
        print("Wait time  =", wait_time / timesteps, "ms")
        print("Frame rate =", 1000*timesteps/(move_time + draw_time + event_time + wait_time), "fps")
    pygame.quit() # quit nicely, so the program window doesn't hang

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        pygame.quit()
        raise e
           