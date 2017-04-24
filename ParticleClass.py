# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 15:52:16 2017

@author: Leslie Murphy, Patrick Marquard, Ian Gorman"""

from vec2d import Vec2d

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