from __future__ import annotations
from math import degrees
import math
from typing import Callable, List, Tuple
from numpy import arange

import pygame
from valet.obstacle import Obstacle
from valet.vehicles.vehicle import Vehicle
from valet.states.ackermann_drive import AckermannDriveState

ACKERMAN_DRIVE_SPRITE = "./valet/assets/ackermann_robot.png"

class Ackermann(Vehicle, pygame.sprite.Sprite):

    def __init__(
        self,
        state: AckermannDriveState,
        pixels_to_meter : int
    ):
        self.pixels_to_meter = pixels_to_meter
        self.state = state
        self.image_shape = (137, 100)
        self.image : pygame.Surface = None
        self.rect : pygame.rect = None
        self.surface : pygame.Surface = None
        self.render()
    
    def render(self):
        if self.image is None:
            self.image = pygame.image.load(ACKERMAN_DRIVE_SPRITE).convert_alpha()
            self.rect = self.image.get_rect(center=(50,50))
        self.surface = pygame.transform.rotate(self.image, -1*degrees(self.state.theta))
        xy = (self.state.x*self.pixels_to_meter, self.state.y*self.pixels_to_meter)
        self.rect = self.surface.get_rect(center=xy)
        self.mask = pygame.mask.from_surface(self.surface)
    
    def blit(self, surface : pygame.Surface):
        surface.blit(self.surface, self.rect)

    def collision_check(self, obstacle : Obstacle):
        # First we test the rects - if they overlap,
        # we can then spend the time doing a finer degree
        # of checking 
        if obstacle.rect.colliderect(self.rect):
            offset = (self.rect[0] - obstacle.rect[0], self.rect[1] - obstacle.rect[1])
            obstacle_mask = pygame.mask.from_surface(obstacle.surface)
            collisions = obstacle_mask.overlap(self.mask, offset)
            if collisions is not None and len(collisions) > 0:
                return True

        return False

    def draw_vehicle(self, display_surface: pygame.Surface, render: Callable) -> Vehicle:
        position : Tuple[int, int] = None
        orientation : float = None
        while orientation is None:
            render()
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
        render()

        x = position[0] / self.pixels_to_meter
        y = position[1] / self.pixels_to_meter
        theta = math.radians(-1*orientation)

        self.state = AckermannDriveState((x, y), theta, 0.0)

    def clone(self) -> Ackermann:
        state = self.state.clone()
        return Ackermann(state, self.pixels_to_meter)
    