from valet.game import Game
from valet.vehicles.skid_drive import SkidDrive

print("launching")
game = Game((800, 800), 24, SkidDrive)
print("inting")
game.init()
# game.on_render()
print("drawing")
game.draw_obstacles()
game.save_map("./tmp.map")
print("Map saved to tmp.map")