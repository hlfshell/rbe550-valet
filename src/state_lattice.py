from typing import Callable, Dict, List, Optional
from states import State
from queues import AStar


class StateLattice():

    def __init__(
        self,
        initial_state: State,
        goal_state: State,
        heuristic_cost_function: Optional[Callable] = None
    ):
        self.start = initial_state
        self.goal = goal_state
        self.queue = AStar()
        self.steps_taken = 0
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

    def search(self) -> List[State]:
        path: Optional[List[State]] = None
        self.queue.push(self.start)

        while path == None:
            path = self.step()
        
        return path
    
    def step(self) -> Optional[List[State]]:
        self.steps_taken += 1

        # If the queue is empty, game over - no path exists!
        if len(self.queue) == 0:
            raise Exception("no path to the goal exists")

        # Get the next candidate from our current state
        current = self.queue.pop()

        # If our current state is our goal state, we've hit our
        # goal, we're done! Let's build the path...
        if self.goal == current:
            path: List[State] = [current]
            while True:
                next : State = self.parents[current]
                path.append(next)
                if self.start == next:
                    return path
        
        if self.start == current:
            current_cost = 0
        else:
            parent = self.parents[current]
            current_cost = parent.transition_cost(current)
            self.costs[current] = current_cost
        
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
        neighbors = current.get_neighbors()
        for neighbor in neighbors:
            if neighbor not in self.parents:
                self.parents[neighbor] = current
                heuristic_cost = 0
                if self.heuristic_cost_function is not None:
                    heuristic_cost = self.heuristic_cost_function(neighbor)
                
                transition_cost = current.transition_cost(neighbor)

                neighbor_cost = current_cost + transition_cost
                self.costs[neighbor] = neighbor_cost

                total_cost = neighbor_cost + heuristic_cost

                self.queue.push(neighbor, total_cost)