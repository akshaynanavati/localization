from heapq import heappush, heappop
import math
import random
import Tkinter as tk

from constants import (
    DELAY,
    GRID_WIDTH,
    HEIGHT,
    INITIAL_PARTICLE_NOISE,
    MOVE_UNITS,
    N_PARTICLES,
    ROBOT_RADIUS,
    WIDTH,
)
from robot import (
    Particle,
    Robot,
)
import util


def obstacles_to_grid(obstacles):
    """
    Transforms a list of obstacles into an m x n grid where 1 represents the cell is covered by
    an obstacle and 0 represents no obstacle. m and n are derived by the canvas height/width and
    grid width. The assumption is that the obstacles fall precisely on grid lines.
    """
    m = HEIGHT / GRID_WIDTH
    n = WIDTH / GRID_WIDTH
    grid = [[0 for _ in xrange(n)] for _ in xrange(m)]
    for o in obstacles:
        jstart = o.x1 / GRID_WIDTH
        jend = o.x2 / GRID_WIDTH
        istart = o.y1 / GRID_WIDTH
        iend = o.y2 / GRID_WIDTH

        for i in xrange(istart, iend):
            for j in xrange(jstart, jend):
                grid[i][j] = 1

    return grid


class Map(object):
    """
    The simulation map. Keeps track of obstacles, particles, and the robot.
    """
    def __init__(
        self,
        obstacles=None,
        robot=None
    ):
        self.obstacles = obstacles or []
        self.grid = obstacles_to_grid(self.obstacles)
        self.robot = robot or Robot()
        self.paused = True
        self.particles = [
            Particle(
                x=robot.x + random.gauss(0.0, INITIAL_PARTICLE_NOISE),
                y=robot.y + random.gauss(0.0, INITIAL_PARTICLE_NOISE),
                orientation=self.robot.orientation
            )
            for i in xrange(N_PARTICLES)
        ]
        self.path = None

        master = tk.Tk()
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.canvas.focus_set()
        self.canvas.bind('<space>', self.toggle_pause)
        self.canvas.bind("<Button-1>", self.mouse_clicked)
        self.timer_fired()

    def timer_fired(self):
        """
        The main game loop.
        """
        self.robot.compute_lidar_measurement(self.obstacles)
        self.render()
        if self.path and not self.paused:
            i, j = self.path[0]
            x = j * GRID_WIDTH + GRID_WIDTH / 2
            y = i * GRID_WIDTH + GRID_WIDTH / 2
            if util.dist((x, y), self.robot.coord()) < MOVE_UNITS:
                self.path = self.path[1:]
                i, j = self.path[0]
                x = j * GRID_WIDTH + GRID_WIDTH / 2
                y = i * GRID_WIDTH + GRID_WIDTH / 2
            delta_theta = self.robot.turn_towards(x, y)
            self.robot.move(MOVE_UNITS)
            for p in self.particles:
                p.turn(delta_theta)
                p.move(MOVE_UNITS)
                if 0 < p.x < WIDTH and 0 < p.y < HEIGHT:
                    p.compute_lidar_measurement(self.obstacles)
            self.particles = util.resample_particles(self.particles, self.robot.lidar_measurements)
        self.canvas.after(DELAY, self.timer_fired)

    def render(self):
        """
        Renders the map and all entities on it (robot, obstacles, particles, path).
        """
        self.canvas.delete(tk.ALL)
        self.render_grid()

        for o in self.obstacles:
            o.render(self.canvas)

        self.robot.render(self.canvas)
        self.robot.render_lidar(self.canvas)

        for p in self.particles:
            p.render(self.canvas)

        self.render_path()

    def render_path(self):
        if self.path is None:
            return

        self.canvas.create_line(
            self.robot.x + ROBOT_RADIUS * math.cos(self.robot.orientation),
            self.robot.y + ROBOT_RADIUS * math.sin(self.robot.orientation),
            self.path[1][1] * GRID_WIDTH + GRID_WIDTH / 2,
            self.path[1][0] * GRID_WIDTH + GRID_WIDTH / 2,
            width=4,
            fill='slate gray'
        )
        i1, j1 = self.path[1]
        for i in xrange(2, len(self.path)):
            i2, j2 = self.path[i]
            x1 = j1 * GRID_WIDTH + GRID_WIDTH / 2
            y1 = i1 * GRID_WIDTH + GRID_WIDTH / 2
            x2 = j2 * GRID_WIDTH + GRID_WIDTH / 2
            y2 = i2 * GRID_WIDTH + GRID_WIDTH / 2
            self.canvas.create_line(x1, y1, x2, y2, width=4, fill='slate gray')
            i1, j1 = i2, j2

    def render_grid(self):
        for i in xrange(HEIGHT / GRID_WIDTH):
            for j in xrange(WIDTH / GRID_WIDTH):
                y1 = i * GRID_WIDTH
                y2 = i * GRID_WIDTH + GRID_WIDTH
                x1 = j * GRID_WIDTH
                x2 = j * GRID_WIDTH + GRID_WIDTH
                if self.grid[i][j] == 1:
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill='orange')
                elif self.grid[i][j] == 2:
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill='yellow')
                else:
                    self.canvas.create_rectangle(x1, y1, x2, y2, fill='white')

    def toggle_pause(self, event):
        self.paused = not self.paused

    def mouse_clicked(self, event):
        if self.path is None:
            self.path = self.find_path(event.x, event.y)

    def run_loop(self):
        tk.mainloop()

    def find_path(self, x, y):
        """
        Uses A* to determine a path from the robot to x, y. It uses the grid (rather than the
        obstacles) to determine the path.

        The assumption is that x, y is not inside an obstacle and that a valid path exists from the
        robot to x, y.
        """
        i_target = y / GRID_WIDTH
        j_target = x / GRID_WIDTH
        i_start = self.robot.y / GRID_WIDTH
        j_start = self.robot.x / GRID_WIDTH
        came_from = {}
        came_from[(i_start, j_start)] = None
        to_visit = []
        seen = set()
        heappush(to_visit, (0, (i_start, j_start, 0)))
        while len(to_visit) > 0:
            cur_i, cur_j, dist = heappop(to_visit)[1]
            seen.add((cur_i, cur_j))
            if cur_i == i_target and cur_j == j_target:
                break
            for delta_i, delta_j in (
                [0, 1],
                [1, 0],
                [-1, 0],
                [0, -1],
            ):
                i = cur_i + delta_i
                j = cur_j + delta_j
                if (
                    0 <= i < HEIGHT / GRID_WIDTH and
                    0 <= j < WIDTH / GRID_WIDTH and
                    (i, j) not in seen and
                    self.grid[i][j] == 0
                ):
                    dist += 1
                    target_dist = math.sqrt(math.pow(i - i_target, 2) + math.pow(j - j_target, 2))
                    came_from[(i, j)] = (cur_i, cur_j)
                    heappush(to_visit, (dist + target_dist, (i, j, dist)))

        path = []
        i, j = i_target, j_target
        while came_from[(i, j)]:
            path.insert(0, (i, j))
            i, j = came_from[(i, j)]

        path.insert(0, (i, j))
        return path
