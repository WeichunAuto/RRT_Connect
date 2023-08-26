import copy
import math
import os
import pickle

import numpy as np


class Obstacles():

    def __init__(self, position=(0, 0), sides=8, area=10):
        self.position = position
        self.sides = sides
        self.area = area

        self.x = []
        self.y = []
        self.vertexs = []

        self.initialize()

    def initialize(self):
        side_ratio = np.random.uniform(low=0.9, high=1.1)
        # side_ratio = 0.9
        # print(f'ration = {side_ratio}')
        side_lengths = self.calculate_side_lengths(self.area, side_ratio)

        angle = 2 * np.pi / self.sides
        angles = np.arange(0, 2 * np.pi, angle)

        x = self.position[0] + side_lengths * np.cos(angles)
        y = self.position[1] + side_lengths * np.sin(angles)

        self.x = x
        self.y = y

        for i in range(len(x)):
            self.vertexs.append((x[i], y[i]))

    def calculate_side_lengths(self, area, ratio):
        # Calculate the side lengths based on the desired area and ratio
        side_lengths = np.zeros(self.sides)
        side_lengths[0] = np.sqrt(area * ratio)
        for i in range(1, self.sides):
            side_lengths[i] = side_lengths[i - 1] / ratio
        a = np.random.randint(low=math.ceil(self.sides / 2), high=self.sides, size=(math.floor(self.sides / 2)),
                              dtype='int')
        for j in range(math.floor(self.sides / 2)):
            side_lengths[j] = side_lengths[a[j]]
        return side_lengths

    def get_obstacle_points(self):
        return self.vertexs

    def get_obstacle_axis(self):
        x = copy.deepcopy(self.x)
        x = np.append(x, [x[0]], axis=0)

        y = copy.deepcopy(self.y)
        y = np.append(y, [y[0]], axis=0)
        return x, y

    def get_obstacle_edges(self):
        points = self.get_obstacle_points()
        edges = []
        for index, point in enumerate(points):
            edges.append([point, points[0 if index + 1 == len(points) else index + 1]])
        # print(f'edges = {edges}')
        return edges


def generate_obstacles(numbers=1):
    '''
    generate a certain number of obstacles
    '''
    obs_list = []

    for i in range(numbers):
        init_pos = 1 if np.random.uniform(-1, 1) > 0 else -1
        pos_x = i * 30 * init_pos
        pos_y = np.random.uniform(-300, 300)

        slide = np.random.randint(3, 9)

        area = np.random.randint(300, 400)

        obs = Obstacles(position=(pos_x, pos_y), sides=slide, area=area)

        obs_list.append(obs)

    return obs_list


file_name = "ObsMap/obstacles.pickle"


def save_obstacles(obstacles):
    # obstacles = generate_obstacles(10)
    file = open(file_name, 'wb')
    pickle.dump(obstacles, file)
    file.close()


def load_obstacles(n):
    obstacles = None
    if os.path.exists(file_name):
        file = open(file_name, 'rb')
        obstacles = pickle.load(file)
    else:
        obstacles = generate_obstacles(n)
        save_obstacles(obstacles)

    return obstacles
