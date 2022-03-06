
from typing import Callable, Dict, List, Optional, Tuple

from valet.queues import AStar
from valet.state_lattice import StateLattice
from valet.states import State


class Planner():

    def __init__(
        self,
        initial_state: State,
        goal_state: State,
        display,
        collision_detection: Callable
    ):
        self.display = display
        self.start = initial_state
        self.goal = goal_state
        self.queue = AStar()
        self.steps_taken = 0

        self.parents: Dict[Tuple[int, int], State] = {}

        self.collision_detection = collision_detection
    
    def search(self) -> List[State]:
        path: Optional[List[State]] = None
        self.queue.push(self.start)

        while path == None:
            path = self.step()
        print("COMPLETE", path)
        print(self.steps_taken)
        
        return path
    
    def step(self) -> Optional[List[State]]:
        self.steps_taken += 1

        if self.steps_taken == 50_000:
            raise "Excessive search steps to discover path"
        
        current = self.queue.pop()

        if self.goal == current:
            print("goal found")
        
        if self.start == current:
            current_cost = 0
        else:
            current_cost = self.costs[current]
        
