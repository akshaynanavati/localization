import math

DELAY = 50  # Delay in microseconds between iterations of run loop
GRID_WIDTH = 40  # Height and width of each grid square in pixels
HEIGHT = 600  # Height of the canvas in pixels
INITIAL_PARTICLE_NOISE = 20  # Noise for seeding the initial particles with respect to the robot
MOVE_NOISE = 0.05  # Noise when moving the robot and particles
MOVE_UNITS = 5  # Units to move per game cycle
N_MEASUREMENTS = 9  # Number of lidar measurements to take
N_PARTICLES = 200  # Number of particles to use
PARTICLE_RADIUS = 1  # Radius of particle when rendering it
ROBOT_RADIUS = 10  # Radius of robot when rendering it
SENSOR_NOISE = 1.0  # Noise for lidar sensor calcuations
TURN_NOISE = 0.025  # Noise when turning the robot
WIDTH = 800  # Width of the canvas in pixels

# Derived constants
MAP_DIAGONAL = math.sqrt(math.pow(WIDTH, 2) + math.pow(HEIGHT, 2))
GRID_DIAGONAL = math.sqrt(2 * math.pow(GRID_WIDTH, 2))
