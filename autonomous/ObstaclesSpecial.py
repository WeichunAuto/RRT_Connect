from autonomous.Obstacles import Obstacles


class ObstaclesSpecial():

    def __init__(self, position=(0, 0), style='U'):
        self.style = style
        self.position = position
        self.vertexs = []

        self.initialize()

    def initialize(self):
        if self.style == 'U':
            p1 = (-50, 50)
            p2 = (-50, -50)
            p3 = (50, -50)
            p4 = (50, 50)
            p5 = (40, 50)
            p6 = (40, -40)
            p7 = (-40, -40)
            p8 = (-40, 50)
            p9 = (-50, 50)

            self.vertexs = [p1, p2, p3, p4, p5, p6, p7, p8, p9]

        if self.style == 'S':
            p1 = (self.position[0]-10, self.position[1]+100)
            p2 = (self.position[0]-10, self.position[1]-100)
            p3 = (self.position[0]+10, self.position[1]-100)
            p4 = (self.position[0]+10, self.position[1]+100)
            p5 = (self.position[0]-10, self.position[1]+100)
            #
            self.vertexs = [p1, p2, p3, p4, p5]

    def get_obstacle_points(self):
        return self.vertexs

    def get_obstacle_axis(self):
        x = []
        y = []
        for p in self.vertexs:
            x.append(p[0])
            y.append(p[1])
        return x, y

    def get_obstacle_edges(self):
        points = self.get_obstacle_points()
        edges = []
        for index, point in enumerate(points):
            edges.append([point, points[0 if index + 1 == len(points) else index + 1]])
        return edges

def generate_obstacles(n, style=''):
    if style == 'S':
        obs_list = []
        x_pos = 0
        y_pos = 0
        for i in range(n):
            x_pos += 50
            if i % 2 == 0:
                y_pos = 0
            else:
                y_pos = 30

            obs = ObstaclesSpecial(position=(x_pos, y_pos), style=style)
            obs_list.append(obs)

        return obs_list