from __future__ import annotations
from typing import List, Optional, Tuple
from math import atan, cos, sin, sqrt, tan, atan, radians, pi

from numpy import arange
import pygame
from valet.states.states import State

class AckermannDriveState(State):

    def __init__(
        self,
        xy: Tuple[float, float],
        theta: float,
        psi: float,
        exact=False
    ):
        self.x = xy[0]
        self.y = xy[1]
        self.theta = theta
        self.psi = psi
        if not exact:
            x, y, theta, psi = self.get_rounded()
            self.x = x
            self.y = y
            self.theta = theta
            self.psi = psi
        self.theta = self.theta % (2*pi)
        self.psi = self.psi  % (2*pi)

        # Ackermann rules
        self.L = 2.8
        self.max_velocity = 5
        self.psi_max = radians(60)

    def get_neighbors(self, increment : float, time_increment : float) -> List[State]:
        v_max = self.max_velocity
        psi_increment = radians(10)

        neighbors : List[AckermannDriveState] = []

        v_increment = 0.5
        # for v in arange(-v_max, self.psi_max + v_increment, v_increment):
        for v in [-v_max, v_max]:
            for psi in arange(-self.psi_max, self.psi_max + psi_increment, psi_increment):
                thetadot = (v/self.L) * tan(psi)
                thetadelta = thetadot * time_increment
                thetadelta = thetadelta % (2*pi)
                if thetadelta > pi:
                    thetadelta = (2*pi) - thetadelta
                    thetadelta = -1 * thetadelta
                theta = self.theta + thetadelta
                xdot = v * cos(theta)
                xdelta = xdot * time_increment
                ydot = v * sin(theta)
                ydelta = ydot * time_increment

                state = self.delta(xdelta, ydelta, thetadelta)
                state.psi = psi
                neighbors.append(state)
        
        return neighbors

    @classmethod
    def draw_path(self, display : pygame.Surface, start : AckermannDriveState, end : AckermannDriveState, time_increment : float, pixels_per_meter : int):
        states = AckermannDriveState.get_transitions_states(start, end, time_increment, 10)
        first = states.pop(0)
        second  = states.pop(0)
        while True:
            firstxy = (first.x*pixels_per_meter, first.y*pixels_per_meter)
            secondxy = (second.x*pixels_per_meter, second.y*pixels_per_meter)
            pygame.draw.line(display, (0, 255, 0), firstxy, secondxy, width=2)
            first = second
            if len(states) == 0:
                break
            second = states.pop(0)
        pygame.display.update()

    def delta(self, deltax : float, deltay : float, deltatheta : float, exact :  bool = False) -> AckermannDriveState:
        x = self.x + deltax
        y = self.y + deltay
        theta = self.theta + deltatheta
        return AckermannDriveState((x, y), theta, self.psi, exact=exact)

    def get_delta(self, to: AckermannDriveState) -> Tuple[float, float, float]:
        xdelta = to.x - self.x
        ydelta = to.y - self.y
        thetadelta = to.theta - self.theta

        if abs(thetadelta) > pi:
            thetadelta = (2*pi) - abs(thetadelta)

        return (xdelta, ydelta, thetadelta)

    def transition_cost(self, to: State):
        # Steering / Theta penalty
        theta_difference = abs(self.theta - to.theta) % (2*pi)
        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference
        theta_penalty = 0 * theta_difference
        
        # Distance Penalty
        distance_penalty = self.distance_between(to)
        cost = distance_penalty + theta_penalty
        return cost

    def distance_between(self, other : State) -> float:
        return sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def connects(self, other : AckermannDriveState, time_increment : float) -> bool:
        thetadelta = self.theta - other.theta
        theta = other.theta
        if theta == 0:
            theta = 0.001

        xdelta = self.x - other.x
        xdot = xdelta / time_increment
        ydelta = self.y - other.y
        ydot = ydelta / time_increment
        if xdelta != 0:
            v = xdot / cos(other.theta)
        else:
            v = ydot / sin(other.theta)

        if v > self.max_velocity or v < -self.max_velocity:
            return False
        
        if xdelta != 0:
            psi = atan((thetadelta * self.L)/(xdelta/cos(theta)))
        else:
            psi = atan((thetadelta * self.L)/(ydelta/sin(theta)))
        
        psi = psi % (2*pi)
        if abs(psi) > pi:
            psi = -1*((2*pi) - abs(psi))

        if psi >= -radians(self.psi_max) and psi <= radians(self.psi_max):
            return True
        else:
            return False

    def clone(self) -> State:
        return AckermannDriveState(
            (self.x, self.y),
            self.theta,
            self.psi,
            exact = True
        )

    def get_rounded(self) -> Tuple[float, float, float, float]:
        x = round(self.x, 2)
        y = round(self.y, 2)
        theta = round(self.theta, 1)
        psi = round(self.psi, 1)
        return (x, y, theta, psi)

    def goal_check(self, other:AckermannDriveState) -> bool:
        if other is None:
            return False
        
        distance = self.distance_between(other)
        theta_distance = abs(self.theta - other.theta)
        if theta_distance > pi:
            theta_distance = (2*pi) - theta_distance

        return distance <= 0.25 and theta_distance < pi/8


    def __eq__(self, other: AckermannDriveState) -> bool:
        if other is None:
            return False
        distance = self.distance_between(other)
        theta_difference = abs(self.theta - other.theta)
        # theta_difference = 0
        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference
        # return distance < 0.05 and theta_difference < 0.05
        return distance < 0.5 and theta_difference < 0.05

    def __hash__(self) -> int:
        x = self.x
        y = self.y
        theta = self.theta
        return hash((x, y, theta))
    
    def __str__(self) -> str:
        return f"x: {self.x} y: {self.y} theta: {self.theta} psi: {self.theta}"

    def __lt__(self, other: AckermannDriveState) -> str:
        x, y, theta, psi = self.get_rounded()
        ox, oy, otheta, opsi = other.get_rounded()
        return (x, y, theta, psi) < (ox, oy, otheta, opsi)
