from math import degrees, sqrt
import math
from typing import Callable, Dict, List, Optional
from uuid import UUID

import pygame
from valet.states import SkidDriveState, State
from valet.queues import AStar
from valet.vehicles.skid_drive import SkidDrive


class StateLattice():

    def __init__(
        self,
        initial_state: State,
        goal_state: State,
        time_increment: float,
        increment: float,
        display,
        collision_detection: Callable,
        heuristic_cost_function: Optional[Callable] = None,
    ):
        self.display = display
        self.start = initial_state
        self.goal = goal_state
        self.queue = AStar()
        self.steps_taken = 0
        self.time_increment = time_increment
        self.increment = increment
        # The parents dict tracks how we reach a given state
        # so that we can rebuild a path during path planning
        self.parents: Dict[State, State] = {}
        # costs is a dict that tracks the cost to reach a
        # given state
        self.costs : Dict[State, float] = {}
        self.costs[self.start] = 0
        # The heuristic cost function solves the h(x), and
        # determines if we're utilizing Dijkstra's or A*
        # It is with this that we can develop different
        # strategies for penalizing behavior to reach our
        # goal faster.
        self.heuristic_cost_function = heuristic_cost_function
        self.collision_detection = collision_detection

    def search(self) -> List[State]:
        path: Optional[List[State]] = None
        self.queue.push(self.start)

        while path == None:
            path = self.step()
        print("done", path)
        print("steps", self.steps_taken)
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
        if current.connects(self.goal, time_increment=self.time_increment):
            self.parents[self.goal] = current
            current = self.goal

        # If our current state is our goal state, we've hit our
        # goal, we're done! Let's build the path...
        # current_thetaless = current.delta(0, 0, 0)
        # current_thetaless.theta = self.goal.theta
        # if self.goal == current_thetaless:
        if self.goal == current:
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
        distance = sqrt((current.x-self.goal.x)**2 + (current.y-self.goal.y)**2)
        close = distance < (2 * self.time_increment)
        neighbors = current.get_neighbors(
            self.increment,#5 if not close else 1,
            self.time_increment,#0.5 if not close else 0.2,
            close=close
        )
        for neighbor in neighbors:
            # If we have already reached this state, we don't
            # need to retread over this ground
            # found = False
            # for node in self.parents.values():
            #     if neighbor == node:
            #         found = True
            #         break
            if neighbor not in self.parents:
                # collisions detection
                xy = (neighbor.x, neighbor.y)
                orientation = degrees(neighbor.theta)
                position = (xy[0]*100, xy[1]*100)
                tmp_vehicle = SkidDrive(
                        position, orientation
                    )
                if self.collision_detection(
                    tmp_vehicle
                ):
                    continue

                self.parents[neighbor] = current
                heuristic_cost = 0
                if self.heuristic_cost_function is not None:
                    heuristic_cost = self.heuristic_cost_function(neighbor, self.goal)

                transition_cost = current.transition_cost(neighbor)

                neighbor_cost = current_cost + transition_cost
                self.costs[neighbor] = neighbor_cost

                total_cost = neighbor_cost + heuristic_cost
                # if close:
                #     total_cost = neighbor_cost + heuristic_cost
                # else:
                #     total_cost = heuristic_cost

                self.queue.push(neighbor, total_cost)

                # Draw a dot for its current spot
                pos = (neighbor.x, neighbor.y)
                pos = (pos[0]*100, pos[1]*100)
                self.display.fill((0, 0, 0), (pos, (2, 2)))

        pygame.event.get()
        pygame.display.update()