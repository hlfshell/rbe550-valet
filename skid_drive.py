from valet.game import Game
from valet.states.skid_drive import SkidDriveState
from valet.vehicles.ackermann import Ackermann
from valet.vehicles.skid_drive import SkidDrive


game = Game((800, 800), 100, Ackermann)
game.init()
vehicle = SkidDrive(SkidDriveState((0,0), 0), 100)
game.load_map("./maps/skid_drive.map")
game.set_vehicle_spawn(vehicle)

while True:
    game.set_goal(vehicle)
    game.plan()
