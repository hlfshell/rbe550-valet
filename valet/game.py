from random import choice
from time import sleep
from typing import Tuple
import pygame
from pygame.locals import K_d, K_RETURN
import math
from valet.states import SkidDriveState, State
from valet.vehicles.skid_drive import SkidDrive
from valet.vehicles.vehicle import Vehicle
# from valet.vehicles.vehicle import Vehicle
from valet.state_lattice import StateLattice

RED = (255, 0, 0)
WHITE = (255, 255, 255)


class Game:

    def __init__(
        self,
        window_size: Tuple[int, int] = (800, 800)
    ):
        self.window_size = window_size

        self._running = False
        self._display_surface = None
        self._vehicle : SkidDrive = None

        self._frame_per_sec = pygame.time.Clock()
        self._fps = 60

    def init(self):
        pygame.init()
        self._running = True 
        self._display_surface = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption("Valet")
        self.on_render()
        pygame.display.update()

    def on_event(self, event):
        if event.type == pygame.QUIT:
            self._running = False

    def on_render(self):
        self._display_surface.fill(WHITE)

        if self._vehicle is not None:
            self._vehicle.render()
            self._display_surface.blit(self._vehicle.surface, self._vehicle.rect)
        
        pygame.display.update()

    def user_input_state(self) -> Tuple[Tuple[int, int], float]:
        position : Tuple[int, int] = None
        orientation : float = None
        while orientation is None:
            self.on_render()
            if position is not None:
                pygame.draw.line(self._display_surface, RED, position, pygame.mouse.get_pos(), width=3)
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
        self.on_render()
        return position, orientation

    def set_vehicle_spawn(self):
        position, orientation = self.user_input_state()
        # position! (178, 300) -0.5070290609147747
        # position = (178, 300)
        # orientation = -0.507
    
        print("position!", position, orientation)
        # state = SkidDriveState((position), orientation)
        self._vehicle = SkidDrive(position, orientation)
        print("start set", self._vehicle.state)
        self.on_render()

    def set_goal(self):
        position, orientation = self.user_input_state()
        # position! (485, 305) -0.8951737102110684
        # position = (485, 305)
        # orientation = -0.895
        # print("goal position!", position, orientation)
        # state = SkidDriveState((position), orientation)
        # state = self.state
        # self._goal = self._vehicle.state.delta(3.5, 0, 0)
        # self._goal = self._vehicle.state.delta(3.5, 1, 0)
        self._goal = SkidDrive(position, orientation).state
        print("goal set", self._goal)
        self.on_render()

    def hold(self):
        while True:
            self.on_render()
            pygame.display.update()
            self._frame_per_sec.tick(self._fps)

    def plan(self):
        planner = StateLattice(
            self._vehicle.state,
            self._goal,
            self._display_surface,
            heuristic_cost_function=self.hueristic
        )
        path = planner.search()
        print(path)
        print(len(path))
        for state in path:
            self._vehicle.state = state
            self.on_render()
            pygame.display.update()
            sleep(0.1)
        return
    
    def hueristic(self, target):
        distance = math.sqrt((target.x-self._goal.x)**2 + (target.y-self._goal.y)**2)
        theta_difference = self._goal.theta-target.theta
        theta_penalty = (theta_difference % (2*math.pi)) ** 2
        return distance + theta_penalty

    def next_state(self):
        print(">>", self._vehicle.state)
        while True:
            for event in pygame.event.get():
                left : bool
                right : bool
                left, _, right = pygame.mouse.get_pressed()
                mousebuttondown = event.type == pygame.MOUSEBUTTONDOWN
                if mousebuttondown:
                    deltas = self._vehicle.state.generate_deltas()
                    picked = choice(deltas)
                    print("Picked", picked)
                    self._vehicle.state = self._vehicle.state.delta(picked[0], picked[1], picked[2])
                    print(">>", self._vehicle.state)
            self.on_render()