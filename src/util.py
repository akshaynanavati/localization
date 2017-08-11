import math
import random

from constants import (
    GRID_WIDTH,
    HEIGHT,
    N_PARTICLES,
    WIDTH,
)


def gaussian(sigma, mu, x):
    """
    Given a mean mu and standard deviation sigma, determins the gaussian probability of the
    value x.
    """
    sigma_sq = math.pow(sigma, 2)
    den = math.sqrt(2 * math.pi * sigma_sq)
    num = math.exp(-math.pow(x - mu, 2) / 2 / sigma_sq)
    return num / den


def dist(p1, p2):
    """
    Determines the straight line distance between two points p1 and p2 in euclidean space.
    """
    d = math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))
    return d


def get_edges(i, j):
    """
    Converts cell at i, j into 4 line segments to be rendered in euclidean space.
    """
    x1, y1 = j * GRID_WIDTH, i * GRID_WIDTH
    x2, y2 = (j + 1) * GRID_WIDTH, (i + 1) * GRID_WIDTH
    return (
        ((x1, y1), (x1, y2)),
        ((x1, y1), (x2, y1)),
        ((x1, y2), (x2, y2)),
        ((x2, y1), (x2, y2)),
    )


def resample_particles(particles, measurements):
    """
    Given particles and measurements taken from the robot, resamples the particles biasing those
    particles with measurements close to the robot's measurements. We only consider particles
    for which x, y is in our canvas.

    We construct a probability distribution ws with relative probabilities - e.g. if the
    distributation is [4, 1, 2], the second index should be half as likely on average to
    get selected as the third and 1/4th as likely as the first index.

    In order to sample, we imagine our distribution is a circular buffer (in this case with
    circumference 7) where the first segment is an arc of length 4, second segment is an arc of
    length 1 and the third segment is an arc of length 2. We pick a random index to start at (let's
    say 1) and imagine our point is at the start of the first arc. We then slide over by a random
    amount (let's say 2). The arc where our point lands is the particle we select (in this case
    the particle at index 2). We continue this process till we have our desired number of
    particles.
    """
    ps = []
    ws = [
        p.lidar_weight(measurements) for p in particles
        if 0 < p.x < WIDTH and 0 < p.y < HEIGHT
    ]
    mw = max(ws)
    beta = 0.0
    nws = len(ws)
    i = int(random.random() * nws) % nws
    for _ in xrange(N_PARTICLES):
        beta += random.random() * 2.0 * mw
        while beta > ws[i]:
            beta -= ws[i]
            i = (i + 1) % nws
        ps.append(particles[i].clone())
    return ps


def dist_to_line(p, theta, q1, q2):
    """
    Returns the distance between p and the intersection point with the line segment q1 -> q2 in
    the theta direction or None if there is no point of collision.
    """
    p1 = p
    p2 = (p[0] + math.cos(theta), p[1] + math.sin(theta))
    ua_num = (q2[0] - q1[0]) * (p1[1] - q1[1]) - (q2[1] - q1[1]) * (p1[0] - q1[0])
    ua_den = (q2[1] - q1[1]) * (p2[0] - p1[0]) - (q2[0] - q1[0]) * (p2[1] - p1[1])
    if ua_den == 0:
        return None
    ua = 1.0 * ua_num / ua_den

    ub_num = (p2[0] - p1[0]) * (p1[1] - q1[1]) - (p2[1] - p1[1]) * (p1[0] - q1[0])
    ub_den = (q2[1] - q1[1]) * (p2[0] - p1[0]) - (q2[0] - q1[0]) * (p2[1] - p1[1])
    if ub_den == 0:
        return None
    ub = 1.0 * ub_num / ub_den

    if 0 <= ua and 0 <= ub <= 1:
        return dist(
            p,
            (p1[0] + ua * math.cos(theta), p1[1] + ua * math.sin(theta))
        )
    return None
