import pygame
from math import sin, cos, radians
import numpy as np
from common import *

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

    def init(self):
        pygame.init()

    def drawArm(self, angles):
        return 0
    
    def drawPole(self, eef_point, pcx_point):
        self.clearScreen()
        self.drawOnFore(pygame.draw.line, (0,0,200), self.center+self.scale*pygame.Vector2(eef_point[0], -eef_point[1]), self.center+self.scale*pygame.Vector2(pcx_point, 0), 1)
        self.merge()
    
    def run(self, eef_data, pcx_data):
        clock = pygame.time.Clock()

        # Create frames
        frames = []

        for i, eef_point in enumerate(eef_data):
            self.drawPole(eef_point, pcx_data[i])
            frames.append(self.display.copy())
            
        # # To save frames to files:
        # for i in range(0,360,1):
        #     pygame.image.save(frames[i], 'output/frame_'+str(i)+'.png')

        # Traverse frames
        i = 0
        logger.info("pygame visuals loop started")
        while True:
            try:
                event = pygame.event.poll()
                if event.type == pygame.QUIT:
                    break
                self.display.blit(frames[i], (0,0))
                pygame.display.update()  
                pygame.time.delay(100)
                i = (i + 1) % (len(frames))
            except Exception as e:
                print(e)
                pygame.quit()
                break
        pygame.quit()
        logger.info("pygame.quit()")
    
    def to_pygame(coords, height):
        """Convert coordinates into pygame coordinates (lower-left => top left)."""
        return (coords[0], height - coords[1])
        

    def drawOnFack(self, drawfunc, *args):
        drawfunc(self.background, *args)

    def drawOnFore(self, drawfunc, *args):
        drawfunc(self.foreground, *args)

    def clearScreen(self):
        self.foreground.fill((0,0,0))

    def merge(self):
        self.display.blit(self.background,(0,0))
        self.display.blit(self.foreground,(0,0))