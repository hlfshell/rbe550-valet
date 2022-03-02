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
    def get_neighbors(self) -> List[State]:
        pass

    @abstractmethod
    def transition_cost(to: State):
        pass

    @abstractmethod
    def connects(self, other : State) -> bool:
        pass

    @abstractmethod
    def distance_between(self, other : SkidDriveState) -> float:
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

    def get_neighbors(self, close = False, time_increment=0.1) -> List[State]:
        neighbors = []

        uruls = []
        increment = 0.5

        urul_max = 20.0
        for urd in arange(-0.5*urul_max, urul_max+increment, increment):
            for url in arange(-0.5*urul_max, urul_max+increment, increment):
                if url < 0 and urd > 0:
                    continue
                elif url > 0 and urd < 0:
                    continue
                elif url == 0 and urd == 0:
                    continue
                uruls.append((urd, url))

        # For each urul combo, we calculate delta x, y, and theta:
        r = .1 # .2 diameter
        L = 0.4 # justify
        for urul in uruls:
            ur = urul[0]
            ul = urul[1]
            thetadot = (r/L) * (ur-ul)
            thetadelta = thetadot * time_increment
            theta = self.theta + thetadelta

            xdot = (r/2)*(ur+ul)*cos(theta)
            ydot = (r/2)*(ur+ul)*sin(theta)
            xdelta = xdot * time_increment
            ydelta = ydot * time_increment

            state = self.delta(xdelta, ydelta, thetadelta)
            neighbors.append(state)
        return neighbors

    def connects(self, other : SkidDriveState, time_increment=0.1) -> bool:
        # First, is it within a distance to even be possible?
        max_distance = 2 * time_increment # 2m/s
        
        if self.distance_between(other) > max_distance:
            return False

        # Now that we know it's possible, calculate the UL/UR needed
        # to get to that spot. If they are within our acceptable range
        # (IE -10 to -20 per second) then we're set!
        xdelta = other.x - self.x
        thetadelta = other.theta - self.theta

        xdot = xdelta / time_increment
        thetadot = thetadelta / time_increment

        r = .1 # .2 diameter
        L = 0.4

        Ul = ((2*xdot)/(r*cos(other.theta))) - ((thetadot * L)/r)
        Ur = ((thetadot*L)/r) + Ul
        
        if Ul >= -10 and Ul <= 20 and Ur >= -10 and Ur <= 20:
            return True
        else:
            return False

    def delta(self, deltax: float, deltay: float, deltatheta: float) -> SkidDriveState:
        x = self.x + deltax
        y = self.y + deltay
        theta = self.theta + deltatheta
        return SkidDriveState((x, y), theta, parent=self.id)
    
    def transition_cost(self, to: SkidDriveState) -> float:
        theta_difference = abs(self.theta - to.theta) % (2*pi)
        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference
        theta_penalty = 0 * theta_difference
        # theta_penalty = 0.5 * ((self.theta-to.theta)%(2*pi))**2
        distance_penalty = sqrt((self.x-to.x)**2 + (self.y-to.y)**2)
        # theta_penalty = 0
        return distance_penalty + theta_penalty
    
    def distance_between(self, other : SkidDriveState) -> float:
        return sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def get_rounded(self) -> Tuple[float, float, float]:
        x = round(self.x, 2)
        y = round(self.y, 2)
        theta = round(self.theta, 1)
        return (x, y, theta)

    def __eq__(self, other: State) -> bool:
        if other is None:
            return False
        # return self.x == other.x and self.y == other.y and self.theta == other.theta
        distance = sqrt((self.x-other.x)**2+(self.y-other.y)**2)
        theta_difference = abs(self.theta - other.theta) % (2*pi)
        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference
        # return distance < 0.1
        return distance < 0.05 and theta_difference < 0.05
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