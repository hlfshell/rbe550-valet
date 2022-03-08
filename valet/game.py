from time import sleep
from typing import List, Tuple
from numpy import arange
import pygame
from pygame.locals import K_w, K_a, K_s, K_d
import math
from valet.obstacle import Obstacle
from valet.states import SkidDriveState, State
from valet.vehicles.vehicle import Vehicle
from valet.vehicles.skid_drive import SkidDrive
from valet.state_lattice import StateLattice

RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)


class Game:

    def __init__(
        self,
        window_size: Tuple[int, int],
        pixels_per_meter: int,
    ):
        self.window_size = window_size
        self.pixels_per_meter = pixels_per_meter

        self._running = False
        self._display_surface = None
        self._vehicle : Vehicle = None

        self._frame_per_sec = pygame.time.Clock()
        self._fps = 60
        self.obstacles : List[Obstacle] = []
        self.time_interval = 0.5

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

        for obstacle in self.obstacles:
            obstacle.render()
            self._display_surface.blit(obstacle.surface, obstacle.rect)

        if self._vehicle is not None:
            self._vehicle.render()
            self._display_surface.blit(self._vehicle.surface, self._vehicle.rect)
        
        pygame.display.update()

    def input_obstacle(self) -> List[Tuple[int, int]]:
        points = []

        while True:
            self.on_render()
            draw_points = points.copy()
            while len(draw_points) >= 2:
                pygame.draw.line(self._display_surface, RED, draw_points[0], draw_points[1], width=3)
                draw_points.pop(0)
            if len(points) > 0:
                pygame.draw.line(self._display_surface, RED, points[-1], pygame.mouse.get_pos(), width=3)

            pygame.display.update()
            self._frame_per_sec.tick(self._fps)

            for event in pygame.event.get():
                left : bool
                right : bool
                left, _, right = pygame.mouse.get_pressed()
                mousebuttondown = event.type == pygame.MOUSEBUTTONDOWN

                if mousebuttondown and left:
                    points.append(pygame.mouse.get_pos())
                elif mousebuttondown and right:
                    if len(points) < 3:
                        return []
                    else:
                        return points

    def set_vehicle_spawn(self, vehicle : Vehicle):
        vehicle.draw_vehicle(self._display_surface, self.on_render)
        self._vehicle = vehicle
        self.on_render()

    def set_goal(self, vehicle_example):
        vehicle = vehicle_example.clone()
        collision = True
        while collision is True:
            vehicle.draw_vehicle(self._display_surface, self.on_render)
            collision = self.collision_detection(vehicle)

        self._goal = vehicle.state
        self.on_render()

    def draw_obstacles(self):
        while True:
            points = self.input_obstacle()
            if len(points) == 0:
                return

            self.obstacles.append(Obstacle(points))

    def collision_detection(self, vehicle: SkidDrive) -> bool:
        mask = None

        for obstacle in self.obstacles:
            # First we test the rects - if they overlap,
            # we can then spend the time doing a finer degree
            # of checking 
            if obstacle.rect.colliderect(vehicle.rect):
                if mask == None:
                    mask = pygame.mask.from_surface(vehicle.surface)
                offset =  (vehicle.rect[0]-obstacle.rect[0], vehicle.rect[1]-obstacle.rect[1])
                obstacle_mask = pygame.mask.from_surface(obstacle.surface)
                collisions = obstacle_mask.overlap(mask, offset)
                if collisions is not None and len(collisions) > 0:
                    return True

        return False

    def plan(self):
        planner = StateLattice(
            self._vehicle.state,
            self._goal,
            self.time_interval,
            self._display_surface,
            self.pixels_per_meter,
            self.collision_detection,
        )
        path = planner.search()

        self.draw_path(path)
        sleep(1)
        self.animate_path(path)

    def animate_path(self, draw_path: List[State]):
        path = draw_path.copy()
        start = path.pop(0)
        full_path = [start]
        second = path.pop(0)
        while True:
            xdelta, ydelta, thetadelta = start.get_delta(second)
            xdot = xdelta/self.time_interval
            ydot = ydelta/self.time_interval
            thetadot = thetadelta/self.time_interval
            for i in arange(0, self.time_interval * self._fps):
                xdelta = xdot * (1/self._fps)
                ydelta = ydot * (1/self._fps)
                thetadelta = thetadot * (1/self._fps)
                inbetween_state = full_path[-1].delta(xdelta, ydelta, thetadelta, exact=True)
                full_path.append(inbetween_state)
            full_path.append(second)
            start = second
            if len(path) == 0:
                break
            second = path.pop(0)

        for state in full_path:
            self._vehicle.state = state
            self.on_render()
            pygame.display.update()
            self._frame_per_sec.tick(self._fps)

    def draw_path(self, path: List[SkidDriveState], color: Tuple[int, int, int]=(0, 255, 0)):
        drawn = path.copy()
        first = drawn.pop(0)
        second  = drawn.pop(0)
        while True:
            firstxy = (first.x*100, first.y*100)
            secondxy = (second.x*100, second.y*100)
            pygame.draw.line(self._display_surface, color, firstxy, secondxy, width=3)
            first = second
            if len(drawn) == 0:
                break
            second = drawn.pop(0)
        pygame.display.update()
    
    def heuristic_two(self, target : State, goal : State):
        distance = target.distance_between(goal)
        theta_difference = abs((goal.theta-target.theta)%(2*math.pi))
        return 2*distance + 4*theta_difference

    def drive(self):
        while True:
            rotation = 0
            translation = 0
            xdelta = 0
            ydelta = 0
            for event in pygame.event.get():
                pressed_keys = pygame.key.get_pressed()
                if pressed_keys[K_a]:
                    rotation = math.pi * 0.125
                elif pressed_keys[K_d]:
                    rotation = -math.pi *0.125
                
                if pressed_keys[K_w]:
                    translation = 0.2
                elif pressed_keys[K_s]:
                    translation = -0.1
                    
                thetadelta = rotation
                theta = self._vehicle.state.theta + thetadelta
                xdelta = (0.1/2) * translation*math.cos(theta)
                ydelta = (0.1/2) * translation*math.sin(theta)

            new_state = self._vehicle.state.delta(xdelta, ydelta, rotation)
            self._vehicle.state = new_state

            self.collision_detection(self._vehicle)

            self.on_render()
            pygame.display.update()
            self._frame_per_sec.tick(self._fps)