import math

import numpy as np
from scipy.special import comb


class Tools():

    # Calculate the coordinate of projection point
    @staticmethod
    def np_point_on_line(startpoint, endpoint, targetpoint):
        """
        Two points determine a straight line.

        startpoint: a ndarray format, two dimational array, np.array([[6, 6]])

        endpoint: a ndarray format, two dimational array, np.array([[12, 7]])

        targetpoint: a ndarray format, two dimational array, np.array([[2, 1]])

        Return a ndarray, two dimational
        """

        ap = targetpoint - startpoint
        ab = endpoint - startpoint

        epsilon = 1e-8  # Small value to avoid division by zero
        denominator = np.sum(np.multiply(ab, ab), axis=1, keepdims=True)
        denominator = np.where(denominator == 0, epsilon, denominator)  # Replace zeros with epsilon

        t = np.sum(np.multiply(ap, ab), axis=1, keepdims=True) / denominator

        result = startpoint + np.multiply(t, ab)

        return result

    # Calculate the coordinate of the intersection point of two straight lines
    @staticmethod
    def cross_point(line1, line2):
        # 取直线坐标两点的x和y值
        x1 = line1[0][0]
        y1 = line1[0][1]
        x2 = line1[1][0]
        y2 = line1[1][1]

        x3 = line2[0][0]
        y3 = line2[0][1]
        x4 = line2[1][0]
        y4 = line2[1][1]

        # L2直线斜率不存在操作
        if (x4 - x3) == 0:
            k2 = None
            b2 = 0
            x = x3

            # L1垂直于Y轴
            if x1 == x2:
                return None

            # 计算k1,由于点均为整数，需要进行浮点数转化
            k1 = (y2 - y1) * 1.0 / (x2 - x1)
            # 整型转浮点型是关键
            b1 = y1 * 1.0 - x1 * k1 * 1.0
            y = k1 * x * 1.0 + b1 * 1.0
        elif (x2 - x1) == 0:
            k1 = None
            b1 = 0
            x = x1
            k2 = (y4 - y3) * 1.0 / (x4 - x3)
            b2 = y3 * 1.0 - x3 * k2 * 1.0
            y = k2 * x * 1.0 + b2 * 1.0
        else:
            # 计算k1,由于点均为整数，需要进行浮点数转化
            k1 = (y2 - y1) * 1.0 / (x2 - x1)
            # 斜率存在操作
            k2 = (y4 - y3) * 1.0 / (x4 - x3)
            # if two straight are parallel, return none
            if k1 == k2:
                return None

            # 整型转浮点型是关键
            b1 = y1 * 1.0 - x1 * k1 * 1.0
            b2 = y3 * 1.0 - x3 * k2 * 1.0
            x = (b2 - b1) * 1.0 / (k1 - k2)
            y = k1 * x * 1.0 + b1 * 1.0
        return (x, y)

    @staticmethod
    def is_legal_point(startpoint, nextpoint, obstacles, safety_radius=2):
        is_legal = True
        is_break = False

        for obstacle in obstacles:

            if is_break is True:
                break

            edges = obstacle.get_obstacle_edges()
            # print(edges)
            line2 = [startpoint, nextpoint]
            # print(f'startpoint = {startpoint}, nextpoint = {nextpoint}')
            for edge in edges:
                # 得到起始点和下一个点连成的线条与障碍物每条边 交叉点坐标， 如果交叉点落在障碍物的某一条边上，则该点非法。
                edge_startpoint = edge[0]
                edge_endpoint = edge[1]
                crosspoint = Tools.cross_point(edge, line2)
                if crosspoint is not None:
                    # calculate the length of edge.
                    edge_lenth = Tools.getDistance(edge_startpoint, edge_endpoint)

                    # calculate the length between crosspoint and the end point of edge.
                    length_cross_edgeend = Tools.getDistance(crosspoint, edge_endpoint)

                    # calculate the length between crosspoint and the start point of edge.
                    length_cross_edgestart = Tools.getDistance(crosspoint, edge_startpoint)

                    if edge_lenth >= length_cross_edgeend and edge_lenth >= length_cross_edgestart:  # 交点 落在 该条边上
                        # 再检查交点 是否落在 startpoint与nextpoint组成的线段上
                        length_start_next = Tools.getDistance(startpoint, nextpoint)
                        length_cross_start = Tools.getDistance(crosspoint, startpoint)
                        length_cross_next = Tools.getDistance(crosspoint, nextpoint)

                        if (
                                length_start_next >= length_cross_start and length_start_next >= length_cross_next):  # 交点在两条线段上
                            is_legal = False
                            is_break = True
                            break

                min_distance = Tools.min_distance_from_pointtoline(edge_startpoint, edge_endpoint, nextpoint)
                if min_distance < safety_radius:  # 如果投影点与目标点之间的距离 小于 安全距离
                    # print(f"min_distance = {min_distance}")
                    is_legal = False
                    is_break = True
                    break

                # 边的两点分别变成 目标点， 在 起始点 与 下一个点 组成直线上的投影
                min_distance = Tools.min_distance_from_pointtoline(startpoint, nextpoint, edge_startpoint)
                if min_distance < safety_radius:
                    is_legal = False
                    is_break = True
                    break

                min_distance = Tools.min_distance_from_pointtoline(startpoint, nextpoint, edge_endpoint)
                if min_distance < safety_radius:
                    is_legal = False
                    is_break = True
                    break

        return is_legal

    # 返回点到障碍物的最短距离
    @staticmethod
    def min_distance_from_pointto_obstacle(point, obstacle):
        edges = obstacle.getObstacleEdges()
        min_distance = 0
        for edge in edges:
            edge_startpoint = edge[0]
            edge_endpoint = edge[1]
            distance = Tools.min_distance_from_pointtoline(edge_startpoint, edge_endpoint, point)
            if min_distance == 0 or min_distance > distance:
                min_distance = distance

        return min_distance

    # 返回点到线段的最短距离
    @staticmethod
    def min_distance_from_pointtoline(startpoint, endpoint, targetpoint):
        min_distance = 0
        edge_lenth = round(np.sqrt((endpoint[1] - startpoint[1]) ** 2 + (endpoint[0] - startpoint[0]) ** 2), 8)

        s_point = np.array([[startpoint[0], startpoint[1]]])
        e_point = np.array([[endpoint[0], endpoint[1]]])
        t_point = np.array([[targetpoint[0], targetpoint[1]]])

        projectpoint = Tools.np_point_on_line(s_point, e_point, t_point)[0]

        # calculate the length between projectpoint and the end point of edge.
        length_project_edgeend = np.sqrt(
            (endpoint[1] - projectpoint[1]) ** 2 + (endpoint[0] - projectpoint[0]) ** 2)

        # calculate the length between projectpoint and the start point of edge.
        length_project_edgestart = np.sqrt(
            (startpoint[1] - projectpoint[1]) ** 2 + (startpoint[0] - projectpoint[0]) ** 2)

        if (round((length_project_edgeend + length_project_edgestart),
                  8) == edge_lenth):  # 投影点 落在 该条边上，最短距离为 目标点 和 投影点 的距离
            min_distance = np.sqrt((targetpoint[1] - projectpoint[1]) ** 2 + (targetpoint[0] - projectpoint[0]) ** 2)

            return min_distance

        else:  # 投影点不在该条边上，最短距离为 边的起点和终点分别与目标点的距离，其中较小者
            length_target_edgeend = round(
                np.sqrt((endpoint[1] - targetpoint[1]) ** 2 + (endpoint[0] - targetpoint[0]) ** 2), 8)
            length_target_edgestart = round(
                np.sqrt((startpoint[1] - targetpoint[1]) ** 2 + (startpoint[0] - targetpoint[0]) ** 2), 8)

            min_distance = length_target_edgeend if length_target_edgeend < length_target_edgestart else length_target_edgestart

            return min_distance

    # 计算 点 到障碍物各个顶点的距离，返回最短距离的那个顶点坐标
    @staticmethod
    def min_distance_from_pointtoobs(point, obstacle):
        vertexes = obstacle.getObstaclePoints()
        # print(vertexes)
        min_distance = 0
        min_vertex = None
        for vertex in vertexes:
            distance = Tools.getDistance(point, vertex)
            if min_distance == 0 or distance < min_distance:
                min_distance = distance
                min_vertex = vertex

        # print(f"min_vertex = {min_vertex}")
        return min_vertex

    # 计算两个坐标点之间的距离，精确到小数点8位
    @staticmethod
    def getDistance(point1, point2):
        return np.sqrt((point1[1] - point2[1]) ** 2 + (point1[0] - point2[0]) ** 2)

    # 求两条方向 向量的夹角
    @staticmethod
    def angle_between_vectors(v1, v2):
        """
        return the angle between the given two vectors.
            参数格式：
            v1 = [(0, 1), (1, 1)]
            v2 = [(1, 1), (0, 0.5)]
        """
        # dx1 = v1[2] - v1[0]
        dx1 = v1[1][0] - v1[0][0]
        # dy1 = v1[3] - v1[1]
        dy1 = v1[1][1] - v1[0][1]
        # dx2 = v2[2] - v2[0]
        dx2 = v2[1][0] - v2[0][0]
        # dy2 = v2[3] - v2[1]
        dy2 = v2[1][1] - v2[0][1]

        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle

        return included_angle

    # 定义搜索空间范围，障碍物上下/左右 最大与最小坐标与安全距离的2倍
    @staticmethod
    def get_search_space(obstacles, departure, destination, safety_radius):
        """
        calculate the search space
        :param obstacles:
        :param departure:
        :param destination:
        :param safety_radius:
        :return: search space = {
                                    'max_left':
                                    'min_right':
                                    'min_top':
                                    'max_down':
                                }
        """
        max_left = None
        min_right = None
        min_top = None
        max_down = None

        for obstacle in obstacles:
            points = obstacle.get_obstacle_points()

            for point in points:

                if max_left is None or max_left < point[0]:
                    max_left = point[0]
                if min_right is None or min_right > point[0]:
                    min_right = point[0]
                if min_top is None or min_top < point[1]:
                    min_top = point[1]
                if max_down is None or max_down > point[1]:
                    max_down = point[1]

        if destination[0] > max_left:
            max_left = destination[0]
        if destination[0] < min_right:
            min_right = destination[0]
        if destination[1] > min_top:
            min_top = destination[1]
        if destination[1] < max_down:
            max_down = destination[1]

        if departure[0] < min_right:
            min_right = departure[0]
        if departure[0] > max_left:
            max_left = departure[0]
        if departure[1] > min_top:
            min_top = departure[1]
        if departure[1] < max_down:
            max_down = departure[1]

        space_buffer = safety_radius * 1

        return {
            'max_left': max_left + space_buffer,
            'min_right': min_right - space_buffer,
            'min_top': min_top + space_buffer,
            'max_down': max_down - space_buffer
        }

    # 保留小数点位数，直接截取位数，不四舍五入
    @staticmethod
    def cut(num, c):
        '''
        Returns the specified number of decimal places without rounding. Example: Tools.cut(2.348, 2) -> return 2.34
        :param num:
        :param c:
        :return:
        '''
        c = 10 ** (-c)
        return (num // c) * c

    @staticmethod
    def find_point_on_vector(vector_A, vector_B, length):
        """
        Given vector AB, there is a point C on the vector AB, and |AC| = length, find the coordinates of point C
        :param vector_A: the coordination of vector OA
        :param vector_B: the coordination of vector OB
        :param length: the length of vector AC, |AC|
        :return: the coordinates of vector OC
        """
        # suppose the coordination vector_AB = (a, b)
        vector_AB = (vector_B[0] - vector_A[0], vector_B[1] - vector_A[1])
        a = vector_AB[0]
        b = vector_AB[1]

        if a == 0:
            x = 0
            y = length
        else:
            x = (a / (np.power(a, 2) + np.power(b, 2)) * (Tools.getDistance((0, 0), (a, b)) * length))
            y = (b / a) * x

        vector_C = (x + vector_A[0], y + vector_A[1])

        return vector_C

    @staticmethod
    def find_point_ahead_vector(vector_A, vector_B, length):
        # suppose the coordination vector_AB = (a, b)
        vector_AB = (vector_B[0] - vector_A[0], vector_B[1] - vector_A[1])
        a = vector_AB[0]
        b = vector_AB[1]
        x = length / Tools.getDistance((0, 0), (a, b)) * a + vector_B[0]
        y = length / Tools.getDistance((0, 0), (a, b)) * b + vector_B[1]
        return (x, y)

    @staticmethod
    def generate_cubic_bezier_curve(control_points, num_points=100):
        """
        generate bezier curve based on the control points
        :param control_points: could be a list with this format [(x0, y0), (x1, y1), (x2, y2)] or
                a ndarray with this format np.array([(x0, y0), (x1, y1), (x2, y2)])
        :param num_points: number of total points on the generated curve
        :return: a ndarray list with 100 points on the generated curve
        """
        if isinstance(control_points, list) is True:  # convert the list to a ndarray
            control_points = np.array(control_points)

        t = np.linspace(0, 1, num_points)
        n = len(control_points) - 1
        curve_points = np.zeros((num_points, 2))

        for i in range(num_points):
            for j in range(n + 1):
                curve_points[i] += control_points[j] * comb(n, j) * (1 - t[i]) ** (n - j) * t[i] ** j

        return curve_points
