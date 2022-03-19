from __future__ import annotations
from math import degrees
import math
from typing import Callable, List, Tuple
import numpy as np

import pygame
from valet.obstacle import Obstacle
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
        if self.image is None:
            self.image = pygame.image.load(ACKERMAN_DRIVE_SPRITE).convert_alpha()
            self.rect = self.image.get_rect(center=(50,50))
            
        # self.surface = pygame.transform.rotate(self.image, -1*degrees(self.state.theta))
        self.surface = pygame.transform.rotate(self.image, -1*degrees(self.state.theta))
        self.xy = (self.state.x*self.pixels_to_meter, self.state.y*self.pixels_to_meter)
        self.rect = self.surface.get_rect(center=self.xy)

        # Calculate where the trailer xyz is
        # trailer_theta = (self.state.trailer_theta + self.state.theta) + (math.pi/2)
        # trailer_theta = self.state.trailer_theta - (math.pi/2)
        trailer_theta = self.state.trailer_theta #+ (math.pi/2)
        xdiff = (5*self.pixels_to_meter) * math.cos(trailer_theta)
        ydiff = (5*self.pixels_to_meter) * math.sin(trailer_theta)
        self.trailer_xy = (self.xy[0] - xdiff, self.xy[1] - ydiff)

        if self.trailer_image is None:
            self.trailer_image = pygame.image.load(TRAILER_SPRITE).convert_alpha()
        trailer_angle = -1*degrees(trailer_theta) #+ (math.pi/2)
        self.trailer_surface = pygame.transform.rotate(self.trailer_image, trailer_angle)
        self.trailer_rect = self.trailer_surface.get_rect(center=self.trailer_xy)
        
        self.mask = pygame.mask.from_surface(self.surface)
        self.trailer_mask = pygame.mask.from_surface(self.trailer_surface)

    def blit(self, surface : pygame.Surface):
        surface.blit(self.surface, self.rect)
        surface.blit(self.trailer_surface, self.trailer_rect)
        pygame.draw.line(surface, (0, 0, 0), self.xy, self.trailer_xy, width=3)

    def collision_check(self, obstacle: Obstacle) -> bool:
        # First we test the rects - if they overlap,
        # we can then spend the time doing a finer degree
        # of checking 
        if obstacle.rect.colliderect(self.rect):
            offset = (self.rect[0] - obstacle.rect[0], self.rect[1] - obstacle.rect[1])
            collisions = obstacle.mask.overlap(self.mask, offset)
            if collisions is not None and len(collisions) > 0:
                return True
        if obstacle.rect.colliderect(self.trailer_rect):
            offset = (self.trailer_rect[0] - obstacle.rect[0], self.trailer_rect[1] - obstacle.rect[1])
            collisions = obstacle.mask.overlap(self.trailer_mask, offset)
            if collisions is not None and len(collisions) > 0:
                return True

        return False

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
        origin = position
        # theta = math.radians(-1*orientation)
        theta = math.radians(-1*orientation)

        self.state = AckermannTrailerState((x, y), theta, 0.0, theta)

    def clone(self) -> AckermannTrailer:
        state = self.state.clone()
        return AckermannTrailer(state, self.pixels_to_meter)
    