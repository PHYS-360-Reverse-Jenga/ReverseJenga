# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 15:52:16 2017

@author: Leslie Murphy, Patrick Marquard, Ian Gorman
"""
import pygame
from vec2d import Vec2d
import math

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
        self.massinv = 1.0/mass
        self.moment = moment
        self.momentinv = 1.0/moment
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