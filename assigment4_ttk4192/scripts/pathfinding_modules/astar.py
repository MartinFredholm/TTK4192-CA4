import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Rectangle
import numpy as np

from utils.grid import Grid
from utils.environment import Environment, Environment_robplan
from utils.cases import TestCase , map_grid_robplan
from utils.utils import distance

from time import time
from math import pi


class Node:
    """ Standard A* node. """

    def __init__(self, cell_id):

        self.cell_id = cell_id
        self.g = None
        self.f = None
        self.parent = None
    
    def __eq__(self, other):

        return self.cell_id == other.cell_id
    
    def __hash__(self):

        return hash((self.cell_id))


class Params:
    """ Store the computed costs. """

    def __init__(self, cell_id, g):

        self.cell_id = cell_id
        self.g = g
    
    def __eq__(self, cell_id):

        return self.cell_id == cell_id
    
    def __hash__(self):

        return hash((self.cell_id))


class Astar:
    """ Standard A*. """

    def __init__(self, grid, start):

        self.grid = grid
        self.start = self.grid.to_cell_id(start)
        self.table = []
    
    def heuristic(self, p1, p2):
        """ Simple Manhattan distance  heuristic. """

        return abs(p2[0]-p1[0]) + abs(p2[1]-p1[1])
    
    def eucledian_heuristic(self, p1, p2):
        """ Eucledian distance heuristic. """
        return distance(p1, p2)
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.cell_id))
            node = node.parent
        
        route.append((self.start))
        
        return list(reversed(route))
    
    def search_path(self, goal):
        """ Search the path by astar. """

        goal = self.grid.to_cell_id(goal)

        if goal in self.table:
            return self.table[self.table.index(goal)].g

        root = Node(self.start)
        root.g = 0
        root.f = root.g + self.heuristic(self.start, goal)

        closed_ = []
        open_ = [root]
    
        while open_:

            best = min(open_, key=lambda x: x.f)
            open_.remove(best)
            closed_.append(best)

            if best.cell_id == goal:
                route = self.backtracking(best)
                self.table.append(Params(goal, best.g))
                return best.g , route

            nbs = self.grid.get_neighbors(best.cell_id)
            for nb in nbs:
                child = Node(nb)
                child.g = best.g + 1
                child.f = child.g + self.heuristic(nb, goal)
                child.parent = best

                if child in closed_:
                    continue

                if child not in open_:
                    open_.append(child)
                
                elif child.g < open_[open_.index(child)].g:
                    open_.remove(child)
                    open_.append(child)
        
        return None


def main_astar(grid_on, start_pos, end_pos):
    print("Running A* pathfinding ...")
    tc = map_grid_robplan()

    env = Environment_robplan(tc.obs)

    grid = Grid(env)

    astar = Astar(grid, start_pos[:2])

    t = time()
    cost, route = astar.search_path(end_pos[:2])

    pts = []
    for x, y in route:
        x = (x+0.5) * grid.cell_size
        y = (y+0.5) * grid.cell_size
        pts.append([x, y])

    WAYPOINTS = np.array(pts)

    def prune_path(WP):
        tol = 0.1
        pts = WP.tolist()
        i = 1
        while i < len(pts) - 1:
            x1, y1 = pts[i-1]
            x2, y2 = pts[i]
            x3, y3 = pts[i+1]
            if (abs(x2-x1)<tol and abs(x3-x2) < tol) or (abs(y2-y1)<tol and abs(y3-y2) < tol):
                pts.pop(i)
            elif x2 == x3 and y2 == y3:
                pts.pop(i)
            else:
                i += 1
        return np.array(pts)
    
    
    WAYPOINTS = prune_path(WAYPOINTS)

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid()
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(start_pos[0], start_pos[1], 'ro', markersize=5)
    ax.plot(end_pos[0], end_pos[1], 'ro', markersize=5)

    lc = LineCollection([pts], linewidth=6)
    ax.add_collection(lc)

    plt.show()

    return WAYPOINTS

