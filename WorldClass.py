# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 15:52:16 2017

@author: Leslie Murphy, Patrick Marquard, Ian Gorman
"""

import pygame

class World:
    def __init__(self, width_pixels, height_pixels, bg_color = (255,255,255)):
        pygame.init()
        self.width = width_pixels
        self.height = height_pixels
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.bg_screen = pygame.Surface((self.width, self.height))
        self.bg_screen.fill(bg_color)
        self.particles = []

    def add(self, particle):
        self.particles.append(particle)
        
    def append(self, particle):
        self.particles.append(particle)
        
    def prepend(self, particle):
        self.particles.insert(0, particle)
        
    def remove(self, particle):
        self.particles.remove(particle)
        
    def display(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.particles:
            if p.visible:
                self.screen.blit(p.image, p.pos - p.offset)
        pygame.display.flip()