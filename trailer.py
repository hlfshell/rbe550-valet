from valet.game import Game
from valet.states.ackermann_trailer import AckermannTrailerState
from valet.vehicles.ackermanntrailer import AckermannTrailer


game = Game((800, 800), 24, AckermannTrailer)
game.init()
vehicle = AckermannTrailer(AckermannTrailerState((0,0), 0, 0, 0), 24)
game.set_vehicle_spawn(vehicle)
game.draw_obstacles()

while True:
    game.set_goal(vehicle)
    game.plan()
