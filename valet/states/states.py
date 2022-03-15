from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple
from math import cos, sin, sqrt, pi
from uuid import uuid4, UUID

from numpy import arange


class State(ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def get_neighbors(self, time_increment : float, close : bool=False) -> List[State]:
        pass

    @abstractmethod
    def transition_cost(to: State):
        pass

    @abstractmethod
    def connects(self, other : State, time_increment : float) -> bool:
        pass

    @abstractmethod
    def distance_between(self, other : State) -> float:
        pass

    @abstractmethod
    def goal_check(self, other : State) -> bool:
        pass

    @abstractmethod
    def clone(self) -> State:
        pass

    @abstractmethod
    def __eq__(self, other: State) -> bool:
        pass

    @abstractmethod
    def __hash__(self) -> int:
        pass
    
    @abstractmethod
    def __str__(self) -> str:
        pass

    @abstractmethod
    def __lt__(self) -> str:
        pass
