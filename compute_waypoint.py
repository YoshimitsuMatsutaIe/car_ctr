"""ウェイポイント計算
xymengのプログラムから拝借
https://github.com/xymeng/rmp_nav/blob/5e04b97b74e601dc11b6b380a95af009f464d87a/rmp_nav/simulation/map_utils.py
"""

from future.utils import iteritems
from past.builtins import xrange
import math
import numpy as np


def a_star(obstacle_map, start_pos, goal_pos):
    height, width = obstacle_map.shape

    # Convert to standard python types. This speeds up path finding significantly.
    obstacle_map = obstacle_map.tolist()
    start_pos = tuple(start_pos)
    goal_pos = tuple(goal_pos)

    def dist_to_goal(x, y):
        # max(x, y) + (sqrt(2)-1)*min(x, y)
        xx = abs(x - goal_pos[0])
        yy = abs(y - goal_pos[1])
        # return np.sqrt(xx**2 + yy**2)
        return max(xx, yy) + (math.sqrt(2) - 1) * min(xx, yy)

    open_dict = {start_pos: (None, 0, dist_to_goal(*start_pos))}
    close_dict = {}

    parents = {}

    def choose_best_node():
        best_g = 1e10
        best_h = 1e10
        best_x = 0
        best_y = 0

        for (x, y), (pa, g, h) in iteritems(open_dict):
            f = g + h
            best_f = best_g + best_h
            # Tie-breaking A*: http://movingai.com/astar.html
            if abs(f - best_f) < 1e-3:
                if best_g < g:
                    best_g = g
                    best_h = h
                    best_x = x
                    best_y = y
            elif f < best_f:
                best_g = g
                best_h = h
                best_x = x
                best_y = y

        return best_x, best_y

    while len(open_dict) > 0:
        x, y = choose_best_node()
        pa, g_score, _ = open_dict.pop((x, y))
        close_dict[(x, y)] = pa

        if (x, y) == goal_pos:
            break

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue

                x2 = dx + x
                y2 = dy + y
                if 0 <= x2 < width and 0 <= y2 < height:
                    if (x2, y2) in close_dict:
                        continue
                    if obstacle_map[y2][x2] > 0:
                        continue
                    if (x2, y2) not in open_dict:
                        # add and compute score
                        g = g_score + math.sqrt(dx ** 2 + dy ** 2)
                        open_dict[(x2, y2)] = (x, y), g, dist_to_goal(x2, y2)
                        parents[(x2, y2)] = (x, y)
                    else:
                        _, g0, h0 = open_dict[(x2, y2)]
                        g = g_score + math.sqrt(dx ** 2 + dy ** 2)
                        if g < g0:
                            open_dict[(x2, y2)] = (x, y), g, dist_to_goal(x2, y2)
                            parents[(x2, y2)] = (x, y)

    if goal_pos not in parents:
        return None

    path = []
    x, y = goal_pos
    while (x, y) != start_pos:
        path.append((x, y))
        x, y = parents[(x, y)]

    return path[::-1]