
from typing import List, Tuple
import pygame

class Obstacle():

    def __init__(self, points : List[Tuple[int, int]]):
        self.rect : pygame.Rect = None
        max_w = max(point[0] for point in points)
        min_w = min(point[0] for point in points)
        max_h = max(point[1] for point in points)
        min_h = min(point[1] for point in points)
        self.w = round(max_w - min_w)
        self.h = round(max_h - min_h)
        # All points are recorded in display pixels,
        # but we need them in the frame of our surface
        self.original_points = points
        self.points = [(point[0]-min_w, point[1]-min_h) for point in points]
        self.center =  (round(max_w - (self.w/2)), round(max_h - (self.h/2)))
        self.surface = pygame.Surface((self.w, self.h))
        self.mask = pygame.mask.from_surface(self.surface)
    
    def render(self):
        self.surface.set_colorkey((0,0,0)) # Set BLACK to alpha key
        pygame.draw.polygon(self.surface, (255, 0, 0), self.points)
        self.rect = self.surface.get_rect(center=self.center)

    def __str__(self) -> str:
        return self.points.__str__()