from __future__ import annotations
from math import degrees
import math
from typing import Callable, Tuple
import numpy as np

import pygame
from valet.vehicles.vehicle import Vehicle
from valet.states.ackermann_trailer import AckermannTrailerState

ACKERMAN_DRIVE_SPRITE = "./valet/assets/ackermann_trailer.png"
TRAILER_SPRITE = "./valet/assets/trailer.png"

class AckermannTrailer(Vehicle, pygame.sprite.Sprite):

    def __init__(
        self,
        state: AckermannTrailerState,
        pixels_to_meter : int
    ):
        self.pixels_to_meter = pixels_to_meter
        self.state = state
        self.image_shape = (137, 100)
        self.image : pygame.Surface = None
        self.trailer_image : pygame.Surface = None
        self.rect : pygame.rect = None
        self.surface : pygame.Surface = None
        self.max_trailer_theta = math.radians(60)
        self.render()
    
    def render(self):
        '''
        1. Load the sprite for the car, rotate it appropriately
        2. Calculate the angle of the bar with the theta, and calculate with a translation matrix the position of the xy of the center of the trailer, since thats calculatable
        3. Load the trailer rotated,place it at calculated x/y
        4. Union the two rects to ge tthe encompassing rect
        5 . Create a usrfae by the size of the rect, and then draw everything into it
        '''

        if self.image is None:
            self.image = pygame.image.load(ACKERMAN_DRIVE_SPRITE).convert_alpha()
            self.rect = self.image.get_rect(center=(50,50))
            
        self.surface = pygame.transform.rotate(self.image, -1*degrees(self.state.theta))
        self.xy = (self.state.x*self.pixels_to_meter, self.state.y*self.pixels_to_meter)
        self.rect = self.surface.get_rect(center=self.xy)

        # Calculate where the trailer xyz is
        trailer_theta = (self.state.trailer_theta + self.state.theta) + (math.pi/2)
        xdiff = (5*self.pixels_to_meter) * math.cos(trailer_theta)
        ydiff = (5*self.pixels_to_meter) * math.sin(trailer_theta)
        self.trailer_xy = (self.xy[0] - xdiff, self.xy[1] - ydiff)

        if self.trailer_image is None:
            self.trailer_image = pygame.image.load(TRAILER_SPRITE).convert_alpha()
        self.trailer_surface = pygame.transform.rotate(self.trailer_image, -1*degrees(trailer_theta))
        self.trailer_rect = self.trailer_surface.get_rect(center=self.trailer_xy)

    def blit(self, surface : pygame.Surface):
        surface.blit(self.surface, self.rect)
        surface.blit(self.trailer_surface, self.trailer_rect)
        pygame.draw.line(surface, (0, 0, 0), self.xy, self.trailer_xy, width=3)
    
    def draw_vehicle(self, display_surface: pygame.Surface, render: Callable) -> Vehicle:
        position : Tuple[int, int] = None
        orientation : float = None

        while orientation is None:
            render()
            # self._frame_per_sec.tick(self._fps)
            if position is not None:
                pygame.draw.line(display_surface, (255, 0, 0), position, pygame.mouse.get_pos(), width=3)
                pygame.display.update()
            for event in pygame.event.get():
                left : bool
                right : bool
                left, _, right = pygame.mouse.get_pressed()
                mousebuttondown = event.type == pygame.MOUSEBUTTONDOWN
                if mousebuttondown and left:
                    if position is None:
                        position = pygame.mouse.get_pos()
                    else:
                        point = pygame.mouse.get_pos()
                        radians = math.atan2(point[0]-position[0], point[1]-position[1])
                        orientation = math.degrees(radians) - 90
                elif mousebuttondown and right and position is not None:
                    position = None

        x = position[0] / self.pixels_to_meter
        y = position[1] / self.pixels_to_meter
        theta = math.radians(-1*orientation)

        self.state = AckermannTrailerState((x, y), theta, 0.0, 0.0)

        tmp_vehicle = AckermannTrailer(self.state, self.pixels_to_meter)
        while True:
            position = pygame.mouse.get_pos()
            trailer_orientation = -1 * math.atan2(point[0]-position[0], point[1]-position[1])
            self.state.trailer_theta = trailer_orientation - self.state.theta
            limit_check = self.state.trailer_theta + (math.pi/2)
            if limit_check > self.max_trailer_theta:
                self.state.trailer_theta = self.max_trailer_theta - (math.pi/2)
            elif limit_check < -self.max_trailer_theta:
                self.state.trailer_theta = -(self.max_trailer_theta - (math.pi/2))
            print(self.state.trailer_theta, math.degrees(self.state.trailer_theta))
            for event in pygame.event.get():
                mousebuttondown = event.type == pygame.MOUSEBUTTONDOWN
                if mousebuttondown and left and tmp_vehicle is not None:
                    return tmp_vehicle
            display_surface.fill((255,255,255)) # white
            tmp_vehicle = AckermannTrailer(self.state, pixels_to_meter=self.pixels_to_meter)
            tmp_vehicle.render()
            tmp_vehicle.blit(display_surface)
            pygame.display.update()
    
    def clone(self) -> AckermannTrailer:
        state = self.state.clone()
        return AckermannTrailer(state, self.pixels_to_meter)
    