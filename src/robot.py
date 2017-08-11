import math
import random

from constants import (
    HEIGHT,
    MOVE_NOISE,
    N_MEASUREMENTS,
    PARTICLE_RADIUS,
    ROBOT_RADIUS,
    SENSOR_NOISE,
    TURN_NOISE,
    WIDTH,
)
import util


class Base(object):
    def __init__(self, x=None, y=None, orientation=None):
        self.x = x or random.random() * WIDTH
        self.y = y or random.random() * HEIGHT
        self.orientation = orientation or random.random() * math.pi * 2
        self.sensor_noise = SENSOR_NOISE
        self.move_noise = MOVE_NOISE
        self.turn_noise = TURN_NOISE
        self.lidar_measurements = [None for _ in xrange(N_MEASUREMENTS)]

    def move(self, units):
        self.orientation += random.gauss(0.0, self.turn_noise)
        units += random.gauss(0.0, self.move_noise)
        deltax = math.cos(self.orientation) * units
        deltay = math.sin(self.orientation) * units
        self.x += deltax
        self.y += deltay
        return self

    def turn(self, theta):
        self.orientation += theta

    def turn_towards(self, x, y):
        """
        Turns such that the robot is facing the point x, y. Returns delta_theta which is
        the amount the robot had to turn to face the new point.
        """
        delta_y = y - self.y
        delta_x = x - self.x
        if delta_x == 0:
            if delta_y > 0:
                theta = math.pi / 2
            else:
                theta = 3 * math.pi / 2
        else:
            theta = math.atan(1.0 * delta_y / delta_x)
        if delta_x < 0:
            theta += math.pi
        delta_theta = theta - self.orientation
        self.orientation = theta
        return delta_theta

    def coord(self):
        return (self.x, self.y)

    def clone(self):
        cls = type(self)
        return cls(self.x, self.y, self.orientation)

    def compute_lidar_measurement(self, obstacles):
        """
        Computes a list of length N_MEASUREMENTS where the ith cell represents the distance to the
        closest obstacle in the i / N_MEASUREMENTS * 2 * pi direction. It computes this by
        determining the collision point between the current point and all segments of all
        obstacles in the theta direction and returning the minimum distance for that direction.
        """
        for k in xrange(N_MEASUREMENTS):
            theta = float(k) / N_MEASUREMENTS * 2 * math.pi
            dists = []
            for o in obstacles:
                for p1, p2 in o.line_segments():
                    dist = util.dist_to_line(self.coord(), theta, p1, p2)
                    if dist is not None:
                        dists.append(dist)
            if not dists:
                for p1, p2 in [
                    ((0, 0), (WIDTH, 0)),
                    ((0, 0), (0, HEIGHT)),
                    ((WIDTH, 0), (WIDTH, HEIGHT)),
                    ((0, HEIGHT), (WIDTH, HEIGHT)),
                ]:
                    dist = util.dist_to_line(self.coord(), theta, p1, p2)
                    if dist is not None:
                        dists.append(dist)
            measurement = min(dists)
            self.lidar_measurements[k] = max(
                0.00001, measurement + random.gauss(0.0, self.sensor_noise)
            )
        return self.lidar_measurements


class Particle(Base):
    def lidar_weight(self, measurements):
        """
        Determines the likelihood of the particle being close to the robot based on the robot's
        lidar measurements.
        """
        p = 1.0
        for i, d in enumerate(self.lidar_measurements):
            if d is None:
                raise RuntimeError("Must compute lidar measurement before calling lidar_weight")
            p *= util.gaussian(d, self.sensor_noise, measurements[i])
        return p

    def render(self, canvas):
        canvas.create_oval(
            self.x - PARTICLE_RADIUS,
            self.y - PARTICLE_RADIUS,
            self.x + PARTICLE_RADIUS,
            self.y + PARTICLE_RADIUS,
            fill='dark green',
            outline='dark green'
        )


class Robot(Base):
    def render(self, canvas):
        canvas.create_oval(
            self.x - ROBOT_RADIUS,
            self.y - ROBOT_RADIUS,
            self.x + ROBOT_RADIUS,
            self.y + ROBOT_RADIUS,
            fill='gold',
        )

    def render_lidar(self, canvas):
        for i, ms in enumerate(self.lidar_measurements):
            if ms is None:
                raise RuntimeError("Must compute lidar measurements before rendering lidar")
            theta = float(i) / N_MEASUREMENTS * 2 * math.pi
            canvas.create_line(
                self.coord()[0],
                self.coord()[1],
                self.coord()[0] + ms * math.cos(theta),
                self.coord()[1] + ms * math.sin(theta),
                fill='red',
            )
