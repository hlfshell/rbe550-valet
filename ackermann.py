from valet.game import Game
from valet.states.ackermann_drive import AckermannDriveState
from valet.vehicles.ackermann import Ackermann


game = Game((800, 800), 24, Ackermann)
game.init()
vehicle = Ackermann(AckermannDriveState((0,0), 0, 0), 24)
game.set_vehicle_spawn(vehicle)
game.draw_obstacles()

while True:
    game.set_goal(vehicle)
    game.plan()