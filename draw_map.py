from valet.game import Game

game = Game((800, 800), 24, None)
game.init()
game.draw_obstacles()
game.save_map("./tmp.map")
print("Map saved to tmp.map")