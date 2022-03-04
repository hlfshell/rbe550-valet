from time import sleep
from valet.game import Game


game = Game()
game.init()
game.set_vehicle_spawn()
game.draw_obstacles()
# game.drive()
while True:
    game.set_goal()
    game.plan()