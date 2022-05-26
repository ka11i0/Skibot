import pygame
from math import sin, cos, radians
import numpy as np

class Window:
    def __init__(self, res, scale = 1):
        """
        A class used to visualize the robot

        ...

        Attributes
        ----------
        res : (int, int)
            window resolution
        center : Vector2
            center of window
        display : pygame.display
            window display object
        background : pygame.Surface
            background surface
        foreground : pygame.Surface
            foreground surface
        scale : int
            scaling variable

        Methods
        -------
        run()
            tbw
        drawArm(v)
            tbw
        drawOnBack(drawfunc, *args)
            tbw
        drawOnFore(drawfunc, *args)
            tbw
        clearScreen()
            tbw
        merge()
            tbw


        """
        self.res = res
        self.center = pygame.Vector2(res[0]/2, res[1]/2)
        self.display = pygame.display.set_mode(res)
        self.background = pygame.Surface(res)
        self.background.fill((255,255,255))
        self.foreground = pygame.Surface(res)
        self.foreground.fill((0,0,0))
        self.foreground.set_colorkey((0,0,0))
        self.scale = scale

    def run(self):
        pygame.init()

    def drawArm(self, v):
        self.clearScreen()
        v1 = self.scale*20*v[0] #limb1 xy
        v2 = self.scale*20*v[1] #limb2 xy
        v2.rotate_ip(-v1.angle_to((0,1)))
        v3 = self.scale*80*v[2] #pole xy
        v3.rotate_ip(-v2.angle_to((0,1)))
        self.drawOnFore(pygame.draw.line, (150,150,150), 
                        pygame.Vector2(0, self.res[1]/2+round(v[3]*100.0)+self.scale*110), 
                        pygame.Vector2(self.res[0], self.res[1]/2+round(v[3]*100.0)+self.scale*110), 
                        1)
        self.drawOnFore(pygame.draw.line, (1,1,1), self.center, self.center+self.scale*80*pygame.Vector2(0, 1), 1)
        self.drawOnFore(pygame.draw.line, (200,0,0), self.center, self.center+v1, 2) # paint limb1
        self.drawOnFore(pygame.draw.line, (0,200,0), self.center+v1, self.center+v1+v2, 2) # paint limb2
        self.drawOnFore(pygame.draw.line, (0,0,200), self.center+v1+v2, self.center+v1+v2+v3, 2) # paint pole
        self.merge()
        

    def drawOnFack(self, drawfunc, *args):
        drawfunc(self.background, *args)

    def drawOnFore(self, drawfunc, *args):
        drawfunc(self.foreground, *args)

    def clearScreen(self):
        self.foreground.fill((0,0,0))

    def merge(self):
        self.display.blit(self.background,(0,0))
        self.display.blit(self.foreground,(0,0))