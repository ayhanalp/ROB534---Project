from rrt_star import RRTStar
from functions import SearchSpace

import math
import random
import numpy as np
import matplotlib.pyplot as plt

import plotly as py
from plotly import graph_objs as go


class MultiagentRRTStar() :
    def __init__(self, dimension_lengths):
        self.num_steps = 10
        self.num_UAVs = 3
        self.con_distance = 10
        self.isConnected = True
        self.paths = dict()
        self.search_points = np.zeros((self.num_steps, self.num_UAVs, 2))
        self.curr_poss = []
        self.dim_lens = dimension_lengths
        self.search_space = SearchSpace(self.dim_lens)

    def calc_distance(self, x1_, y1_, x2_, y2_) :
        return math.sqrt((x2_ - x1_) ** 2 + (y2_ - y1_) ** 2)

    def isNetworkConnected(self, t_i) :

        for UAV_i in range(self.num_UAVs) :
            for other_i in range(self.num_UAVs) :
                if UAV_i != other_i :
                    dist_i = self.calc_distance(self.paths[t_i, UAV_i, 0], self.paths[t_i, UAV_i, 1],
                                                self.paths[t_i, other_i, 0], self.paths[t_i, other_i, 1])

                    if dist_i > self.con_distance :
                        return False
        return True

    def random_search_point_gen(self, t_i) :
        radius = self.con_distance
        point_id = 0

        while True:
            p_i_x_ = random.randint(self.search_space.dimension_lengths[0, 0],
                                    self.search_space.dimension_lengths[0, 1])

            p_i_y_ = random.randint(self.search_space.dimension_lengths[0, 0],
                                    self.search_space.dimension_lengths[0, 1])

            if ([p_i_x_, p_i_y_] not in self.search_points) and (self.search_space.obstacle_free((p_i_x_, p_i_y_))):
                break

        while point_id < self.num_UAVs:
            # random angle
            alpha = 2 * math.pi * random.random()
            # random radius
            r = radius * math.sqrt(random.random())

            p_i_x_ = r * math.cos(alpha) + p_i_x_
            p_i_y_ = r * math.sin(alpha) + p_i_y_

            if ([p_i_x_, p_i_y_] not in self.search_points) and (self.search_space.obstacle_free((p_i_x_, p_i_y_))):
                self.search_points[t_i, point_id, 0] = p_i_x_
                self.search_points[t_i, point_id, 1] = p_i_y_

                point_id += 1

    def multiagent_RRT_star(self):
        paths = dict()
        time_steps = self.num_steps - 1

        X_dimensions = self.dim_lens  # dimensions of Search Space

        # obstacles
        Obstacles = np.array([(20, 20, 40, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)])

        Q = np.array([(2, 1)])  # length of tree edges
        r = 1  # length of smallest edge to check for intersection with obstacles
        max_samples = 1024  # max number of samples to take before timing out
        rewire_count = 32  # optional, number of nearby branches to rewire
        prc = 0.1

        X = SearchSpace(X_dimensions, Obstacles)

        for UAV_id in range(self.num_UAVs):
            self.search_points[0, UAV_id, 0] = 0
            self.search_points[0, UAV_id, 1] = 0

        for step_i in range(time_steps):
            self.random_search_point_gen(step_i + 1)

            for UAV_id in range(self.num_UAVs):
                x_init = (self.search_points[step_i, UAV_id, 0], self.search_points[step_i, UAV_id, 1])
                x_goal = (self.search_points[step_i + 1, UAV_id, 0], self.search_points[step_i + 1, UAV_id, 1])

                rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)

                paths[step_i, UAV_id] = rrt.rrt_star()

        self.paths = paths

        return paths

    #################################


if __name__ == '__main__':
    X_dimensions = np.array([(0, 100), (0, 100)])

    ma_rrt_star = MultiagentRRTStar(X_dimensions)
    paths = ma_rrt_star.multiagent_RRT_star()

    data = []

    colors = ["red", "green", "blue"]
    layout = {'title' : 'Plot', 'showlegend' : False}

    layout['shapes'] = []

    O = np.array([(20, 20, 40, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)])

    for O_i in O :
        # noinspection PyUnresolvedReferences
        layout['shapes'].append(
            {
                'type' : 'rect',
                'x0' : O_i[0],
                'y0' : O_i[1],
                'x1' : O_i[2],
                'y1' : O_i[3],
                'line' : {
                    'color' : 'green',
                    'width' : 4,
                },
                'fillcolor' : 'green',
                'opacity' : 0.70
            },
        )

    for s_i in range(ma_rrt_star.num_steps-1):
        for u_i in range(ma_rrt_star.num_UAVs):
            if ma_rrt_star.paths[s_i, u_i] is not None :
                p_i = ma_rrt_star.paths[s_i, u_i]
                x, y = [], []
                for i in p_i :
                    x.append(i[0])
                    y.append(i[1])
                    plt.scatter(i[0], i[1])
                trace = go.Scatter(
                    x=x,
                    y=y,
                    line=dict(
                        color=colors[u_i],
                        width=4
                    ),
                    mode="lines"
                )

                data.append(trace)

    fig = {'data' : data,
           'layout' : layout}

    py.offline.plot(fig)

    plt.show()
    print(paths)

"""
        for point_id in range(self.num_UAVs):

            while True:
                p_i_x_ = random.randint(self.search_space.dimension_lengths[0, 0],
                                        self.search_space.dimension_lengths[0, 1])

                p_i_y_ = random.randint(self.search_space.dimension_lengths[0, 0],
                                        self.search_space.dimension_lengths[0, 1])

                if [p_i_x_, p_i_y_] not in self.search_points:
                    flag = True
                    for other_id in range(self.num_UAVs):
                        if other_id != point_id:
                            d = self.calc_distance(p_i_x_, p_i_y_, self.search_points[t_i, other_id, 0],
                                                    self.search_points[t_i, other_id, 1])

                            if d > self.con_distance:
                                flag = False
                                break

                    if not flag:
                        continue

                    self.search_points
                    break
"""
