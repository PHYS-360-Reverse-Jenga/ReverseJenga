# -*- coding: utf-8 -*-
"""
Created on Fri Feb 10 15:10:53 2017

@author: Patrick Marquardt, Ian Gorman, Leslie Murphy
"""

import pygame, sys
from pygame.locals import *
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





class Particle:
    def __init__(self, position, velocity, mass, image = None, offset = None):
        self.pos = Vec2d(position)
        self.vel = Vec2d(velocity)
        self.acc = Vec2d(0,0)
        self.image = image
        self.mass = mass
        self.massinv = 1.0/mass
        self.offset = offset
        self.visible = True
        if image != None:
            self.set_image(image, offset)
        
    def set_image(self, image, offset=None):
        self.offset = offset
        if offset == None:
            self.offset = Vec2d(image.get_width()/2, image.get_height()/2)
        self.image = image
        
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
        self.create_origaxes()
        self.angle = angle
        self.angvel = float(angvel)
        self.torque = 0.0
        self.mass = mass
        self.massinv = 1.0/(mass+.000001)
        self.moment = moment
        self.momentinv = 1.0/(moment+.000001)
        self.visible = True
        self.update_points()
        self.update_axes()    
        
    def create_origaxes(self):
        self.origaxes = []
        self.axes = []
        for i in range(len(self.origpoints)):
            a = (self.origpoints[i]-self.origpoints[i-1]).perpendicular_normal()
            self.origaxes.append(a)
            self.axes.append(a)
      
    def update_points(self):
        for i in range(len(self.origpoints)):
            newX = self.origpoints[i].x*math.cos(self.angle) + self.origpoints[i].y*-math.sin(self.angle)
            newY = self.origpoints[i].x*math.sin(self.angle) + self.origpoints[i].y*math.cos(self.angle)
            newPoint =  Vec2d(newX,newY)

            self.points[i] = newPoint + self.pos
                
    def update_axes(self):
        for i in range(len(self.origaxes)):
            #update axes
            newX = self.origaxes[i].x*math.cos(self.angle) + self.origaxes[i].y*-math.sin(self.angle)
            newY = self.origaxes[i].x*math.sin(self.angle) + self.origaxes[i].y*math.cos(self.angle)
            newAxes =  Vec2d(newX,newY)
          
            self.axes[i] = newAxes
                    
    def add_impulse(self, imp, pos):
        self.vel += imp/self.mass
        vec = Vec2d(pos - self.pos)
        self.angvel += (vec.cross(imp))/self.moment
        
    def draw(self, screen):
        if self.visible:
            self.update_points()
            n = len(self.points)
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
        #shapesPlaced = 0
        #font = pygame.font.SysFont('Calibri',25,True,False)
        #text = font.render("Shapes placed: " + shapesPlaced,True,BLACK)
        #self.screen.blit(text,[10,10])
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)
        pygame.display.flip()

    def display_noflip(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)

def update_pos(shapes, dt):
    for p in shapes:
        p.pos   += p.vel*dt
        p.angle += p.angvel*dt

def update_vel(shapes, dt):
    for p in shapes:
        p.vel    += p.force*p.massinv*dt
        p.angvel += (p.torque/(p.moment+.000001))*dt
        
def update_force(shapes, world):
    for p in shapes:
        p.force = Vec2d(0,0)
        p.torque = 0.0
    for p in shapes:
        gravity = Vec2d(0, 0.05)
        p.force += p.mass * gravity

class Wall:
    def __init__(self, normal, position, e=1):
        self.normal = normal
        self.pos = position
        self.e = e
        
def collide(shape1, shape2):
    
    axes = shape1.axes + shape2.axes
    distance = []
    mind = 9999999999
    
    for j,a in enumerate(axes):
        
        s1Min = a.dot(shape1.points[0])
        s1Max = s1Min      
        s2Min = a.dot(shape2.points[0])
        s2Max = s2Min
        
        i1_max = 0
        i1_min = 0
        i2_max = 0
        i2_min = 0
        
        for i in range(len(shape1.points)):
            p1 = (a.dot(shape1.points[i]))
            if(p1< s1Min):
                s1Min = p1
                i1_min = i
            if(p1>s1Max):
                s1Max = p1
                i1_max = i
                
        for i in range(len(shape2.points)):
            p2 = (a.dot(shape2.points[i]))
            if (p2 < s2Min):
                s2Min = p2
                i2_min = i          
            if(p2 > s2Max):
                s2Max = p2
                i2_max = i
        
        distance = min(s1Max-s2Min,s2Max-s1Min)        

        if distance < mind:
            mind = distance
                        
            if mind <= 1e-12:
                return (False, None, None)
                
            if (s1Max - s2Min < s2Max - s1Min):
                disp = -mind*a
                
                if(j < len(shape1.axes)):
                    point = shape2.points[i2_min]
                else:
                    point = shape1.points[i1_max]
            else:
                disp = mind*a    
                
                if(j < len(shape1.axes)):
                    point = shape2.points[i2_max]
                else:
                    point = shape1.points[i1_min]

    return (True, disp, point)    
       
def handle_collisions(shapes, world):
    e = 0.8
    walls = [Wall(Vec2d(0, -1), Vec2d(0,world.height), 0.5), Wall(Vec2d(0, 1), Vec2d(0, 0), 0.5),
             Wall(Vec2d(-1, 0), Vec2d(world.width, 0), 0.5), Wall(Vec2d(1, 0), Vec2d(0, 0), 0.5)]
    for s in shapes:
        s.update_points()
        s.update_axes()
    for i in range(len(shapes)):
        p = shapes[i]

        #Collision with walls
        for wall in walls:
            max_d = -1e15
            wall_point = Vec2d(0,0)
            for t in p.points:
                d = wall.pos.dot(wall.normal) - t.dot(wall.normal)
                if(d>max_d):
                    max_d = d
                    wall_point = t
            if(max_d > 0):
                r = wall_point - p.pos 
                num = (-(1+wall.e)*(p.vel.dot(wall.normal)+p.angvel*(r.cross(wall.normal)))*wall.normal)
                denom = 1/p.mass + (r.cross(wall.normal))**2/(p.moment)       
                j = num/denom                
                p.add_impulse(j, wall_point)
                p.pos += max_d*wall.normal 
                p.update_points()
           
        #start Rectangle collision
        for m in range(len(shapes)):       
            if (m == i):
                continue          
            q = shapes[m]
            displacement = Vec2d(0, 0)
            (collision, displacement, coll_point) = collide(shapes[i], shapes[m])  
            
            if collision:
                #print(m,i, displacement)
                normal = displacement.normalized()              
                #p
                p_r = coll_point - p.pos
                p_num = (-(1 + e)*(p.vel.dot(normal)+p.angvel*(p_r.cross(normal))))
                p_denom = 1/p.mass + (p_r.cross(normal))**2/(p.moment)    
                
                #q
                q_r = coll_point - q.pos
                q_num = (-(1 + e)*(q.vel.dot(normal)+q.angvel*(q_r.cross(normal))))
                q_denom = 1/q.mass + (q_r.cross(normal))**2/(q.moment)

                imp = (p_num - q_num)/(p_denom + q_denom)
                if imp > 0:                            
                    p.add_impulse(imp*normal, coll_point)
                    q.add_impulse(-(imp*normal), coll_point)
                    
                #Handle rectangle depenetration
                p.pos +=  displacement * (p.massinv/(p.massinv + q.massinv))
                q.pos -=  displacement * (q.massinv/(p.massinv + q.massinv))
                p.update_points()
                q.update_points()

    return False

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
    world = World(1200, 800, WHITE)
    screen = pygame.display.set_mode((1200, 800))
    world.display()
    
    # -- Font Stuff -- #
    font = pygame.font.SysFont("Times New Roman", 18)
    fuel_label = font.render("Shapes Placed: ", 1, BLACK)
    
    MAX_RECT_LEN = 400
    MAX_RECT_HEIGHT = 75
    MIN_RECT_LEN = 50
    MIN_RECT_HEIGHT = 25
    
    MAX_TRI_LEN = 300
    MIN_TRI_LEN = 100
    
    moving = []
    clickShapes = []

    clock = pygame.time.Clock()
    done = False
    new_particle_needed = True
    launching = False
    
    density = 1 # mass / area
    timesteps = 0


    while not done:       
        fuel_label = font.render("Shapes Placed: ", 1, BLACK)
        screen.blit(fuel_label, (50, 50))
        
        # Create a new particle to replace one that was launched
        if new_particle_needed:
            new_particle_needed = False
            X,Y = 0,1
            p = pygame.mouse.get_pos()
            mouse_pos = Vec2d(p[X],p[Y])

            #Make Random Rectangle
            randWid = random.randrange(MIN_RECT_LEN, MAX_RECT_LEN)
            randLen = random.randrange(MIN_RECT_HEIGHT, MAX_RECT_HEIGHT)
            randShape = Rectangle((mouse_pos), (0,0), 0, 0, random_color(0,768), 1, randWid, randLen )
            world.add(randShape)
        
        for event in pygame.event.get():    
            if event.type == pygame.QUIT: # Close window clicked
                done = True
                break
            if event.type == pygame.MOUSEBUTTONDOWN:
                #Add Random shape to world
                moving.append(randShape)
                launching = False
                new_particle_needed = True
                
        if not launching:
            pos = Vec2d(pygame.mouse.get_pos())
            randShape.visible = (pygame.mouse.get_focused() and pos.x > 0 and pos.y > 0
                                and pos.x < world.width-1 and pos.y < world.height-1)
            randShape.pos = pos
            
        # Velocity Verlet method
        n = 1
        dt = 1 / n
        collide_max = 1
        for i in range(n):        
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            update_pos(moving, dt)
            collide_count = 0
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            while collide_count < collide_max and handle_collisions(moving, world):
                collide_count += 1
        world.display()     
        clock.tick(30) # wait so that this only updates 60 fps maximum
        
    pygame.quit() # quit nicely, so the program window doesn't hang

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        pygame.quit()
        raise e
           