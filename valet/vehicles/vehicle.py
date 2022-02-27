from abc import ABC, abstractmethod

class Vehicle(ABC):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def render(self):
        pass