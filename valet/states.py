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
    def is_possible(self) -> bool:
        pass

    @abstractmethod
    def get_neighbors(self) -> List[State]:
        pass

    @abstractmethod
    def transition_cost(to: State):
        pass

    @abstractmethod
    def __eq__(self, other: State) -> bool:
        pass


class SkidDriveState(State):

    def __init__(self, xy: Tuple[float, float], theta: float, parent: Optional[UUID] = None):
        super().__init__()
        self.x = xy[0]
        self.y = xy[1]
        self.theta = theta
        x, y, theta = self.get_rounded()
        self.x = x
        self.y = y
        self.theta = theta
        self.theta = theta % (2*pi)
        self.id = uuid4()
        self.parent = parent
    
    def is_possible(self) -> bool:
        return True

    def get_neighbors(self) -> List[State]:
        neighbors = []
        time_delta = 0.1

        uruls = []
        increment = 10
        urul_max = 20.0
        for urd in arange(-0.5*urul_max, urul_max+increment, increment):
            for url in arange(-0.5*urul_max, urul_max+increment, increment):
                # if url < 0 and urd > 0:
                #     continue
                # elif url > 0 and urd < 0:
                #     continue
                if url == 0 and urd == 0:
                    continue
                uruls.append((urd, url))

        # For each urul combo, we calculate delta x, y, and theta:
        r = .1 # .2 diameter
        L = 0.4 # justify
        for urul in uruls:
            ur = urul[0]
            ul = urul[1]
            thetadot = (r/L) * (ur-ul)
            thetadelta = thetadot * time_delta
            theta = self.theta + thetadelta

            xdot = (r/2)*(ur+ul)*cos(theta)
            ydot = (r/2)*(ur+ul)*sin(theta)
            xdelta = xdot * time_delta
            ydelta = ydot * time_delta

            state = self.delta(xdelta, ydelta, thetadelta)
            neighbors.append(state)
        return neighbors

    def delta(self, deltax: float, deltay: float, deltatheta: float) -> SkidDriveState:
        x = self.x + deltax
        y = self.y + deltay
        theta = self.theta + deltatheta
        return SkidDriveState((x, y), theta, parent=self.id)
    
    def transition_cost(self, to: SkidDriveState) -> float:
        theta_penalty = (self.theta-to.theta)**2
        theta_penalty = 0
        return sqrt((self.x-to.x)**2 + (self.y-to.y)**2)+theta_penalty
    
    def get_rounded(self) -> Tuple[float, float, float]:
        x = round(self.x, 2)
        y = round(self.y, 2)
        theta = round(self.theta, 1)
        return (x, y, theta)

    def __eq__(self, other: State) -> bool:
        if other is None:
            return False
        distance = sqrt((self.x-other.x)**2+(self.y-other.y)**2)
        theta_difference = abs(self.theta - other.theta)
        # return distance < 0.1
        return distance < 0.1 and theta_difference < 0.01
        return round(self.x,2) == round(other.x, 2) and \
                round(self.y, 2) == round(other.y, 2)
                # round(self.y, 2) == round(other.y, 2) and \
                # round(self.theta, 2) == round(other.theta, 2)
    
    def __str__(self) -> str:
        return f"x: {self.x} y: {self.y} theta: {self.theta}"
    
    def __hash__(self) -> int:
        # x,y, theta = self.get_rounded()
        x = self.x
        y = self.y
        theta = self.theta
        return hash((x, y, theta))

    def __lt__(self, other: SkidDriveState) -> bool:
        x, y, theta = self.get_rounded()
        ox, oy, otheta = other.get_rounded()
        return (x, y, theta) < (ox, oy, otheta)