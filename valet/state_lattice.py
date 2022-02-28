from math import sqrt
import math
from typing import Callable, Dict, List, Optional
from uuid import UUID

import pygame
from valet.states import State
from valet.queues import AStar


class StateLattice():

    def __init__(
        self,
        initial_state: State,
        goal_state: State,
        display,
        heuristic_cost_function: Optional[Callable] = None
    ):
        self.display = display
        self.start = initial_state
        self.goal = goal_state
        self.queue = AStar()
        self.steps_taken = 0
        # The parents dict tracks how we reach a given state
        # so that we can rebuild a path during path planning
        self.parents: Dict[State, State] = {}
        self.states: Dict[UUID, State] = {}
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

    def search(self) -> List[State]:
        print("seraching")
        path: Optional[List[State]] = None
        self.queue.push(self.start)
        self.states[self.start.id] = self.start

        while path == None:
            path = self.step()
        print("done", path)
        print("steps", self.steps_taken)
        return path
    
    def step(self) -> Optional[List[State]]:
        self.steps_taken += 1
        if self.steps_taken == 50_000:
            print("step", self.steps_taken)
            g = self.goal
            n = self.queue.pop()
            print(g, n, sqrt((g.x-n.x)**2+(g.y-n.y)**2))
            raise "done"

        # If the queue is empty, game over - no path exists!
        if len(self.queue) == 0:
            raise Exception("no path to the goal exists")

        # Get the next candidate from our current state
        current = self.queue.pop()

        # If our current state is our goal state, we've hit our
        # goal, we're done! Let's build the path...
        if self.goal == current:
            print("path found")
            path: List[State] = [current]
            while True:
                current : State = self.parents[current]
                path.insert(0, current)
                if self.start == current:
                    return path
            # print("path found")
            # path: List[State] = [current]
            # while current.parent is not None:
            #     current = self.states[current.parent]
            #     path.insert(0, current)
            # return path
        
        if self.start == current:
            current_cost = 0
        else:
            current_cost = self.costs[current]
            # parent = self.parents[current]
            # parent = self.states[current.parent]
            # current_cost = parent.transition_cost(current)
            # current_cost = 0
            # next_state = parent
            # while next.parent is not None:
            #     current_cost += 
            # self.costs[current] = current_cost
        
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
        neighbors = current.get_neighbors(distance < 0.25)
        for neighbor in neighbors:
            # Is the neighbor explored already?
            # found = False
            # for state in self.states.values():
            #     if state == neighbor:
            #         found = True
            #         break
            # if not found:
            if neighbor not in self.parents:
                self.parents[neighbor] = current
                self.states[neighbor.id] = neighbor
                heuristic_cost = 0
                if self.heuristic_cost_function is not None:
                    heuristic_cost = self.heuristic_cost_function(neighbor)
                # heuristic_cost = 0
                # next_state = neighbor
                # while next_state.parent is not None:
                #     heuristic_cost += 1
                #     next_state = self.states[next_state.parent]
                
                transition_cost = current.transition_cost(neighbor)

                neighbor_cost = current_cost + transition_cost
                self.costs[neighbor] = neighbor_cost

                total_cost = neighbor_cost + heuristic_cost
                # total_cost = heuristic_cost

                self.queue.push(neighbor, total_cost)

                # Draw a dot for its current spot
                pos = (neighbor.x, neighbor.y)
                pos = (pos[0]*100, pos[1]*100)
                self.display.fill((0, 0, 0), (pos, (2, 2)))
            else:
                pos = (neighbor.x, neighbor.y)
                pos = (pos[0]*100, pos[1]*100)
                self.display.fill((0, 255, 0), (pos, (1, 1)))

        pygame.event.get()
        pygame.display.update()
            # self.states[neighbor.id] = neighbor
            # heuristic_cost = 0
            # if self.heuristic_cost_function is not None:
            #     heuristic_cost = self.heuristic_cost_function(neighbor)
            
            # # transition_cost = current.transition_cost(neighbor)
            # transition_cost = sqrt((self.start.x - current.x)**2 + (self.start.y - current.y)**2)

            # neighbor_cost = current_cost + transition_cost
            # self.costs[neighbor] = neighbor_cost

            # total_cost = neighbor_cost + heuristic_cost

            # self.queue.push(neighbor, total_cost)