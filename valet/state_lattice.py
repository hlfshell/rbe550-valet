from math import degrees, sqrt
import math
from time import sleep
from typing import Callable, Dict, List, Optional
from uuid import UUID

import pygame
from valet.states.states import State
from valet.queues import AStar
from valet.vehicles.vehicle import Vehicle


class StateLattice():

    def __init__(
        self,
        initial_state: State,
        goal_state: State,
        time_increment: float,
        display,
        pixels_per_meter : int,
        collision_detection: Callable,
        vehicle_type : Callable
    ):
        self.display : pygame.Surface = display
        self.start = initial_state
        self.goal = goal_state
        self.queue = AStar()
        self.steps_taken = 0
        self.time_increment = time_increment
        self.increment = 5.0
        self.pixels_per_meter = pixels_per_meter
        # The parents dict tracks how we reach a given state
        # so that we can rebuild a path during path planning
        self.parents: Dict[State, State] = {}
        # costs is a dict that tracks the cost to reach a
        # given state
        self.costs : Dict[State, float] = {}
        self.costs[self.start] = 0
        self.collision_detection = collision_detection
        self.vehicle_type : Vehicle = vehicle_type

    def search(self) -> List[State]:
        path: Optional[List[State]] = None
        self.queue.push(self.start)

        while path == None:
            path = self.step()
        return path
    
    def step(self) -> Optional[List[State]]:
        self.steps_taken += 1
        if self.steps_taken == 50_000:
            raise "Excessive search steps to discover path"

        # If the queue is empty, game over - no path exists!
        if len(self.queue) == 0:
            raise Exception("no path to the goal exists")

        # Get the next candidate from our current state
        current = self.queue.pop()

        # Short hop check
        connected = current.connects(self.goal, time_increment=self.time_increment)
        if connected is not None:
            shadow = self.vehicle_type(connected, self.pixels_per_meter)
            if not self.collision_detection(shadow):
                self.parents[connected] = current
                current = connected

        # If our current state is our goal state, we've hit our
        # goal, we're done! Let's build the path...
        # current_thetaless = current.delta(0, 0, 0)
        # current_thetaless.theta = self.goal.theta
        # if self.goal == current_thetaless:
        # if self.goal == current:
        if self.goal.goal_check(current):
            path: List[State] = [current]
            while True:
                current : State = self.parents[current]
                path.insert(0, current)
                if self.start == current:
                    return path
        
        if self.start == current:
            current_cost = 0
        else:
            current_cost = self.costs[current]
        
        # Get each neighbor for the current State and queue
        # them. First, we get a list of potential neighbors
        # that are valid from the current state - this
        # excludes any potential kinematic constraints for
        # the robot itself. We then move through each
        # neighbor and confirm that it does not cause a
        # collision. If it doesn't, we see if it's a neighbor
        # we've considered before. If so, we ignore it. If
        # not, we add it to our queue to consider,
        # calculating its total cost, which includes the
        # heuristic cost as well.
        # distance = sqrt((current.x-self.goal.x)**2 + (current.y-self.goal.y)**2)
        neighbors = current.get_neighbors(
            self.increment,#5 if not close else 1,
            self.time_increment,#0.5 if not close else 0.2,
        )
        for neighbor in neighbors:
            width, height = self.display.get_size()
            max_x = width / self.pixels_per_meter
            max_h = height / self.pixels_per_meter
            # If the neighbor state is negative x/y or has an x/y that goes
            # beyond the total size of our edges, we don't want to consider
            # it. IE don't go out of bounds.
            if neighbor.x < 0 or neighbor.y < 0:
                continue
            if neighbor.x > max_x or neighbor.y > max_h:
                continue

            # If we have already reached this state, we don't
            # need to retread over this ground
            # found = False
            # for node in self.parents.values():
            #     if neighbor == node:
            #         found = True
            #         break
            if neighbor not in self.parents:
                # Collisions detection
                shadow = self.vehicle_type(neighbor, self.pixels_per_meter)
                if self.collision_detection(shadow):
                    continue

                self.parents[neighbor] = current

                distance = neighbor.distance_between(self.goal)
                heuristic_cost = 5 * distance

                transition_cost = current.transition_cost(neighbor)

                neighbor_cost = current_cost + transition_cost
                self.costs[neighbor] = neighbor_cost

                total_cost = neighbor_cost + heuristic_cost

                self.queue.push(neighbor, total_cost)

                # Draw a dot for its current spot
                pos = (neighbor.x, neighbor.y)
                pos = (pos[0]*self.pixels_per_meter, pos[1]*self.pixels_per_meter)
                self.display.fill((0, 0, 0), (pos, (2, 2)))
                # try:
                # AckermannDriveState.draw_path(self.display, current, neighbor, self.time_increment, self.pixels_per_meter)
                # except Exception as e:
                #     print("oh noes", e)
                #     exit()

        pygame.event.get()
        pygame.display.update()
        # sleep(0.5)