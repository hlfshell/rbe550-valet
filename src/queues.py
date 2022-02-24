from queue import PriorityQueue
from states import State

class AStar():

    def __init__(self):
        self.queue = PriorityQueue()
    
    def push(self, state: State, cost=0):
        self.queue.put((cost, state))

    def pop(self) -> State:
        self.queue.get()[1]

    def __len__(self):
        return len(self.queue.queue)