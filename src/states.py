from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List


class State(ABC):

    def __init__(self):
        super().__init__()
    
    @abstractmethod
    def is_possible(self) -> bool:
        pass

    @abstractmethod
    def get_neighbors(self) -> List[State]:
        pass

    @abstractmethod
    def transition_cost(to: State):
        pass

    @abastractmethod
    def __eq__(self, other: State) -> bool:
        pass