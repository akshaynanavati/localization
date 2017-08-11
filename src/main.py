import math

from constants import (
    GRID_WIDTH,
    HEIGHT,
    WIDTH,
)
from map import Map
from obstacle import Obstacle
from robot import Robot


def main():
    obstacles = [
        # Col 1
        Obstacle(0, 0, 2 * GRID_WIDTH, HEIGHT),
        # Col 2
        Obstacle(3 * GRID_WIDTH, 0, 5 * GRID_WIDTH, GRID_WIDTH),
        Obstacle(3 * GRID_WIDTH, 2 * GRID_WIDTH, 5 * GRID_WIDTH, 6 * GRID_WIDTH),
        Obstacle(3 * GRID_WIDTH, 7 * GRID_WIDTH, 5 * GRID_WIDTH, HEIGHT),
        # Col 3
        Obstacle(6 * GRID_WIDTH, GRID_WIDTH, 8 * GRID_WIDTH, HEIGHT - GRID_WIDTH),
        # Col 4
        Obstacle(9 * GRID_WIDTH, 0, 11 * GRID_WIDTH, 2 * GRID_WIDTH),
        Obstacle(9 * GRID_WIDTH, 3 * GRID_WIDTH, 11 * GRID_WIDTH, 10 * GRID_WIDTH),
        Obstacle(9 * GRID_WIDTH, 11 * GRID_WIDTH, 320 + 3 * GRID_WIDTH, HEIGHT),
        # Col 5
        Obstacle(12 * GRID_WIDTH, GRID_WIDTH, 14 * GRID_WIDTH, 5 * GRID_WIDTH),
        Obstacle(12 * GRID_WIDTH, 6 * GRID_WIDTH, 14 * GRID_WIDTH, 8 * GRID_WIDTH),
        Obstacle(12 * GRID_WIDTH, 9 * GRID_WIDTH, 14 * GRID_WIDTH, HEIGHT),
        # Col 6
        Obstacle(15 * GRID_WIDTH, GRID_WIDTH, 17 * GRID_WIDTH, 4 * GRID_WIDTH),
        Obstacle(15 * GRID_WIDTH, 5 * GRID_WIDTH, 17 * GRID_WIDTH, HEIGHT),
        # Col 7
        Obstacle(18 * GRID_WIDTH, GRID_WIDTH, WIDTH, HEIGHT - GRID_WIDTH),
    ]
    robot = Robot(80 + GRID_WIDTH / 2, GRID_WIDTH / 2, math.pi / 2)
    map = Map(
        obstacles=obstacles,
        robot=robot,
    )
    map.run_loop()


if __name__ == '__main__':
    main()
