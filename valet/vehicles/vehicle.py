from __future__ import annotations
from abc import ABC, abstractmethod

import math
from valet.obstacle import Obstacle
from valet.states.states import State
from typing import Callable, List
import pygame


class Vehicle(ABC):

    def __init__(self, state : State, pixels_per_meter : int):
        super().__init__()

    @abstractmethod
    def render(self):
        pass
    
    @classmethod
    def draw_vehicle(self, display_surface : pygame.Surface, render : Callable) -> Vehicle:
        pass

    @abstractmethod
    def clone(self) -> Vehicle:
        pass

    @abstractmethod
    def collision_check(self, obstacle : Obstacle) -> bool:
        pass
