from valet.game import Game
from valet.states.ackermann_drive import AckermannDriveState
from valet.vehicles.ackermann import Ackermann


# game = Game((800, 800), 24, Ackermann)
game = Game((800, 800), 35, Ackermann)
game.init()
vehicle = Ackermann(AckermannDriveState((0,0), 0, 0), 35)
game.load_map("./maps/ackermann.map")
game.set_vehicle_spawn(vehicle)

while True:
    game.set_goal(vehicle)
    game.plan()
