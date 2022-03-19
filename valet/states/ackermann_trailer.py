from __future__ import annotations
from typing import List, Tuple
from math import atan, cos, degrees, sin, sqrt, tan, atan, radians, pi

from numpy import arange
import pygame
from valet.states.states import State

class AckermannTrailerState(State):

    def __init__(
        self,
        xy: Tuple[float, float],
        theta: float,
        psi: float,
        trailer_theta: float,
        exact=False
    ):
        self.x = xy[0]
        self.y = xy[1]
        self.theta = theta
        self.psi = psi
        self.trailer_theta = trailer_theta
        if not exact:
            x, y, theta, psi, trailer_theta = self.get_rounded()
            self.x = x
            self.y = y
            self.theta = theta
            self.psi = psi
            self.trailer_theta = trailer_theta
        self.theta = self.theta % (2*pi)
        self.psi = self.psi  % (2*pi)
        self.trailer_theta = self.trailer_theta % (2*pi)

        # Ackermann rules
        self.L = 2.8
        self.max_velocity = 5
        self.psi_max = radians(60)

    def get_neighbors(self, increment : float, time_increment : float) -> List[State]:
        v_max = self.max_velocity
        psi_increment = radians(10)

        neighbors : List[AckermannTrailerState] = []

        v_increment = 1
        # for v in arange(-v_max, self.psi_max + v_increment, v_increment):
        for v in [-v_max, v_max]:
            for psi in arange(-self.psi_max, self.psi_max + psi_increment, psi_increment):
                thetadot = (v/self.L) * tan(psi)
                thetadelta = thetadot * time_increment
                # thetadelta = thetadelta % (2*pi)
                # if thetadelta > pi:
                #     thetadelta = (2*pi) - thetadelta
                #     thetadelta = -1 * thetadelta
                # if thetadelta < -pi:
                #     thetadelta = (2*pi) + thetadelta
                theta = self.theta + thetadelta
                xdot = v * cos(theta)
                xdelta = xdot * time_increment
                ydot = v * sin(theta)
                ydelta = ydot * time_increment

                # Now solve for the trailer
                trailer_theta_dot = (v/5)*sin(theta-self.trailer_theta)
                trailer_theta_delta = trailer_theta_dot * time_increment

                state = self.delta(xdelta, ydelta, thetadelta, delta_trailer_theta=trailer_theta_delta)
                # state.trailer_theta += trailer_theta_delta
                # Check the difference between the robot theta and trailer theta
                theta_difference = abs(state.theta - state.trailer_theta)
                if theta_difference > (pi/4):
                    # The theta difference is too large - reject it
                    continue
                state.psi = psi
                neighbors.append(state)
        
        return neighbors

    @classmethod
    def draw_path(self, display : pygame.Surface, start : AckermannTrailerState, end : AckermannTrailerState, time_increment : float, pixels_per_meter : int):
        states = AckermannTrailerState.get_transitions_states(start, end, time_increment, 10, 0)
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

    def delta(self, deltax : float, deltay : float, deltatheta : float, delta_trailer_theta : float = 0.0, exact :  bool = False) -> AckermannTrailerState:
        x = self.x + deltax
        y = self.y + deltay
        
        
        # if delta_trailer_theta > pi:
        #     print("2 +", delta_trailer_theta, -((2*pi) - delta_trailer_theta))
        #     delta_trailer_theta = -((2*pi) - delta_trailer_theta)
        # elif delta_trailer_theta < -pi:
        #     print("2 -", delta_trailer_theta, -((2*pi) - delta_trailer_theta))
        #     delta_trailer_theta = -((2*pi) + delta_trailer_theta)
        theta = self.theta + deltatheta
        
        trailer_theta = self.trailer_theta + delta_trailer_theta

        return AckermannTrailerState((x, y), theta, self.psi, trailer_theta, exact=exact)

    def get_delta(self, to: AckermannTrailerState) -> Tuple[float, float, float, float]:
        xdelta = to.x - self.x
        ydelta = to.y - self.y
        thetadelta = to.theta - self.theta
        trailer_thetadelta = to.trailer_theta - self.trailer_theta

        if thetadelta > pi:
            thetadelta = -((2*pi) - thetadelta)
        elif thetadelta < -pi:
            thetadelta = ((2*pi) + thetadelta)
        
        if trailer_thetadelta > pi:
            trailer_thetadelta = -((2*pi) - trailer_thetadelta)
        elif trailer_thetadelta < -pi:
            trailer_thetadelta = ((2*pi) + trailer_thetadelta)

        return (xdelta, ydelta, thetadelta, trailer_thetadelta)

    def transition_cost(self, to: State):
        # Steering / Theta penalty
        theta_difference = abs(self.theta - to.theta) % (2*pi)
        if theta_difference > pi:
            theta_difference = abs(to.theta - self.theta) % (2*pi)
        theta_penalty = 0 * theta_difference
        
        # Distance Penalty
        distance_penalty = self.distance_between(to)
        cost = distance_penalty + theta_penalty
        return cost

    def distance_between(self, other : State) -> float:
        return sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def connects(self, other : AckermannTrailerState, time_increment : float) -> bool:
        distance = self.distance_between(other)
        max_distance = self.max_velocity * time_increment
        if distance > max_distance:
            return False
        
        thetadelta = self.theta - other.theta

        thetadotmax = (self.max_velocity/self.L) * tan(self.psi_max)
        thetadeltamax = abs(thetadotmax) * time_increment
        if abs(thetadelta) > thetadeltamax:
            return False

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

        if abs(v) > self.max_velocity:
            return False
        
        if xdelta != 0:
            psi = atan((thetadelta * self.L)/(xdelta/cos(theta)))
        else:
            psi = atan((thetadelta * self.L)/(ydelta/sin(theta)))
        
        psi = psi % (2*pi)
        if abs(psi) > pi:
            psi = -1*((2*pi) - abs(psi))

        if psi > abs(self.psi_max):
            print("psi is bad", psi, degrees(psi))
            return False
        
        # Check the trailer
        trailer_theta_dot = (v/5)*sin(theta-self.trailer_theta)
        trailer_theta_delta = trailer_theta_dot * time_increment
        trailer_theta = self.trailer_theta + trailer_theta_delta
        theta_difference = abs(theta - trailer_theta)
        if theta_difference > (pi/4):
            return False
        
        print("TRIGGERED")
        return True

    def clone(self) -> State:
        return AckermannTrailerState(
            (self.x, self.y),
            self.theta,
            self.psi,
            self.trailer_theta,
            exact = True
        )

    def get_rounded(self) -> Tuple[float, float, float, float]:
        x = round(self.x, 2)
        y = round(self.y, 2)
        theta = round(self.theta, 1)
        psi = round(self.psi, 1)
        trailer_theta = round(self.trailer_theta, 1)
        return (x, y, theta, psi, trailer_theta)

    def goal_check(self, other:AckermannTrailerState) -> bool:
        if other is None:
            return False
        
        distance = self.distance_between(other)
        theta_distance = abs(self.theta - other.theta)
        if theta_distance > pi:
            theta_distance = abs(other.theta - self.theta)

        return distance <= 0.5 and theta_distance < pi/8


    def __eq__(self, other: AckermannTrailerState) -> bool:
        if other is None:
            return False
        distance = self.distance_between(other)
        theta_difference = abs(self.theta - other.theta)
        # theta_difference = 0
        if theta_difference > pi:
            theta_difference = ((2*pi) - theta_difference)
        return distance < 0.5 and theta_difference < 0.05

    def __hash__(self) -> int:
        x = self.x
        y = self.y
        theta = self.theta
        return hash((x, y, theta))
    
    def __str__(self) -> str:
        theta = degrees(self.theta)
        psi = degrees(self.psi)
        trailer_theta = degrees(self.trailer_theta)
        return f"x: {self.x} y: {self.y} theta: {theta} psi: {psi} trailer theta: {trailer_theta}"

    def __lt__(self, other: AckermannTrailerState) -> str:
        x, y, theta, psi, trailer = self.get_rounded()
        ox, oy, otheta, opsi, otrailer = other.get_rounded()
        return (x, y, theta, psi) < (ox, oy, otheta, opsi)
