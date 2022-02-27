# from src.states import State
from typing import Tuple
from valet.states import SkidDriveState
from valet.vehicles.vehicle import Vehicle
import pygame
from math import radians, degrees

SKID_DRIVE_SPRITE = "./valet/assets/robot.png"

class SkidDrive(Vehicle, pygame.sprite.Sprite):

    def __init__(
        self,
        xy: Tuple[int, int],
        orientation: float,
        pixels_to_meter=100
    ):
        self.pixels_to_meter = pixels_to_meter
        xy = self.pixels_to_position(xy)
        self.state = SkidDriveState(xy, radians(orientation))
        self.image_shape = (137, 100)
        # TODO : resize per pixel conversion
        self.image = pygame.image.load(SKID_DRIVE_SPRITE).convert_alpha()
        self.rect = self.image.get_rect(center=(50, 50))
        self.surface = None

    def render(self):
        self.surface = pygame.transform.rotate(self.image, degrees(self.state.theta))
        # self.image = self.surface # ??
        position = self.position_to_pixels((self.state.x, self.state.y))
        self.rect = self.surface.get_rect(center=position)

    def position_to_pixels(self, xy: Tuple[int, int]) -> Tuple[int, int]:
        '''
        This function returns the resulting conversion
        of a given vehicle-space to pixel-space.
        Orientation remains true.
        '''
        return (xy[0]*self.pixels_to_meter, xy[1]*self.pixels_to_meter)
    
    def pixels_to_position(self, xy: Tuple[int, int]) -> Tuple[int, int]:
        return (xy[0]/self.pixels_to_meter, xy[1]/self.pixels_to_meter)