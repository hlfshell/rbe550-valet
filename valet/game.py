from random import choice
from time import sleep
from typing import List, Tuple
import pygame
from pygame.locals import K_RETURN, K_w, K_a, K_s, K_d
import math
from valet.obstacle import Obstacle
from valet.states import SkidDriveState, State
from valet.vehicles.skid_drive import SkidDrive
from valet.vehicles.vehicle import Vehicle
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
        self.obstacles : List[Obstacle] = []

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

    def input_pose(self) -> Tuple[Tuple[int, int], float]:
        position : Tuple[int, int] = None
        orientation : float = None
        while orientation is None:
            self.on_render()
            self._frame_per_sec.tick(self._fps)
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
        return position, -1*orientation

    def input_polygon(self) -> List[Tuple[int, int]]:
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
                    print("clicked")
                    points.append(pygame.mouse.get_pos())
                elif mousebuttondown and right:
                    print("clocked")
                    if len(points) < 3:
                        return []
                    else:
                        return points

    def set_vehicle_spawn(self):
        position, orientation = self.input_pose()
        # position = (76, 405)
        # orientation = -0.82
        print("start", position, orientation)
        self._vehicle = SkidDrive(position, orientation)
        print("start set", self._vehicle.state)
        self.on_render()

    def set_goal(self):
        collision = True
        while collision is True:
            position, orientation = self.input_pose()
            collision = self.collision_detection(
                SkidDrive(position, orientation)
            )
        # position = (635, 378)
        # orientation = -85.236
        print("goal", position, orientation)
        self._goal = SkidDrive(position, orientation).state
        print("goal set", self._goal)
        self.on_render()

    def draw_obstacles(self):
        while True:
            points = self.input_polygon()
            if len(points) == 0:
                print(self.obstacles)
                return

            self.obstacles.append(Obstacle(points))
        # self.obstacles = [
        #     Obstacle([(247, 214), (274, 530), (581, 509), (534, 154)])
        # ]

    def hold(self):
        while True:
            self.on_render()
            pygame.display.update()
            self._frame_per_sec.tick(self._fps)

    def collision_detection(self, vehicle: SkidDrive) -> bool:
        mask = None

        for obstacle in self.obstacles:
            # First we test the rects - if they overlap,
            # we can then spend the time doing a finer degree
            # of checking 
            if obstacle.rect.colliderect(vehicle.rect):
                # print("rect collision")
                if mask == None:
                    mask = pygame.mask.from_surface(vehicle.surface)
                offset =  (vehicle.rect[0]-obstacle.rect[0], vehicle.rect[1]-obstacle.rect[1])
                obstacle_mask = pygame.mask.from_surface(obstacle.surface)
                collisions = obstacle_mask.overlap(mask, offset)
                if collisions is not None and len(collisions) > 0:
                    # print("COLLISION")
                    return True

        return False

    def plan(self):
        planner = StateLattice(
            self._vehicle.state,
            self._goal,
            self._display_surface,
            self.collision_detection,
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
    
    def hueristic(self, target : State, goal : State):
        distance = target.distance_between(goal)
        theta_difference = goal.theta-target.theta
        theta_penalty = abs(theta_difference % (2*math.pi))
        # theta_penalty = 0
        return 2*(distance + theta_penalty)

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