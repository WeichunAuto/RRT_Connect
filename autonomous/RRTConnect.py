import numpy as np

from autonomous.Tools import Tools
from autonomous.TreeNode import TreeNode


class RRTConnect():

    def __init__(self, departure, destination, obstacles, safeRadius, maxIterations=200):
        self.nodeCount = 1

        self.departure = departure
        self.departureTree = TreeNode(departure[0], departure[1])
        self.treeNodes = [self.departureTree]  # consist of all tree nodes

        self.destination = destination
        self.destinationTree = TreeNode(destination[0], destination[1])

        self.treeTurns = [self.departureTree, self.destinationTree]

        self.maxIterations = min(maxIterations, 5000)

        self.obstacles = obstacles
        self.safeRadius = safeRadius
        self.stepSize = safeRadius * 1.5
        self.targetRadius = self.safeRadius * 5

        self.searchSpace = self.calculate_search_space()

        self.closestNode = None
        self.minDistance = 0  # the minimum distance between a leadpoint and nodes
        self.newPoint_closest = None

        self.connection = False
        self.previousNewTPoint = self.departure
        self.junctionNodes = []

        self.NOS = 1  # Number of solutions to be produced

        self.lastLeadPoint = None


    def grow_motion_path(self, isGreedy=False, isDynamicStep=False):
        """
        Rapidly grow random tree, find some path solutions
        :param isGreedy: apply greedy strategy
        :return:
        """
        for iterate in range(self.maxIterations):

            if iterate >= self.maxIterations:
                break

            if self.newPoint_closest is not None:
                self.previousNewTPoint = self.newPoint_closest

            self.get_legal_children_point(self.treeTurns[0])
            newChildNode = TreeNode(self.newPoint_closest[0], self.newPoint_closest[1])
            newChildNode.parent = self.closestNode
            self.closestNode.children.append(newChildNode)
            self.treeNodes.append(newChildNode)

            leadPoint = self.newPoint_closest

            self.generate_nexttree_nodes(self.treeTurns[1], leadPoint, self.stepSize, isGreedy, isDynamicStep)

            if self.connection is True:
                self.junctionNodes.append(newChildNode)
                print(f"iteration = {iterate}")
                break

            self.treeTurns.reverse()

    def generate_nexttree_nodes(self, tree, leadPoint, stepSize, isGreedy, isDynamicStep):
        """
        generate nodes in next tree
        :param tree:
        :param leadPoint:
        :param stepSize:
        :param isGreedy:
        :return:
        """

        self.minDistance = 0
        self.find_children_point_from_tree(tree, leadPoint, stepSize)
        while 1:
            isLegal = Tools.is_legal_point((self.closestNode.locationX, self.closestNode.locationY),
                                           self.newPoint_closest,
                                           self.obstacles, self.safeRadius)
            if isLegal is not True:
                # the purpose is to set previousNewTPoint = closestNode, when newPoint_closest is illegal
                self.newPoint_closest = (self.closestNode.locationX, self.closestNode.locationY)

                # if the new point is illegal, then reset the stepSize
                if isGreedy is True and isDynamicStep is True:
                    stepSize = self.stepSize

                break

            newChildNode = TreeNode(self.newPoint_closest[0], self.newPoint_closest[1])
            newChildNode.parent = self.closestNode
            self.closestNode.children.append(newChildNode)
            self.treeNodes.append(newChildNode)

            self.try_connect(leadPoint, self.previousNewTPoint, self.newPoint_closest)
            if self.connection is True:
                self.junctionNodes.append(newChildNode)
                break

            if isGreedy is False:
                break
            else:
                if isDynamicStep is True:
                    stepSize += self.stepSize

                self.closestNode = newChildNode
                self.newPoint_closest = self.find_children_point_from_closestnode(self.closestNode, leadPoint, stepSize)

    def try_connect(self, leadPoint, previousNewTPoint, newPoint_closest):
        """
        try to connect lead point and new point or previous new targeted point and new point
        :param leadPoint:
        :param newPoint_closest:
        :return:
        """
        self.lastLeadPoint = leadPoint

        if Tools.is_legal_point(leadPoint, newPoint_closest, self.obstacles,
                                self.safeRadius) is True and Tools.getDistance(leadPoint,
                                                                               newPoint_closest) <= self.targetRadius:
            self.connection = True

        elif Tools.is_legal_point(previousNewTPoint, newPoint_closest, self.obstacles,
                                self.safeRadius) is True and Tools.getDistance(previousNewTPoint,
                                                                               newPoint_closest) <= self.targetRadius:
            self.connection = True

    def get_solution_paths(self):
        """
        all possible path solutions
        :return: return a list with all possible path solutions
        """
        first_tree_path = []
        second_tree_path = []

        first_tree_path.append(self.junctionNodes[0])
        self.append_parent_node(self.junctionNodes[0].parent, first_tree_path)

        second_tree_path.append(self.junctionNodes[1])
        self.append_parent_node(self.junctionNodes[1].parent, second_tree_path)

        first_tree_path.reverse()
        solution_path = first_tree_path
        solution_path.extend(second_tree_path)

        return solution_path

    def append_parent_node(self, node, solution_path):
        '''
        put all the nodes which have a parent node into a solution list
        :param solution_path:
        :param node:
        :return:
        '''
        solution_path.append(node)
        if node.parent is not None:
            self.append_parent_node(node.parent, solution_path)

    def is_reach_destination(self, newPoint):
        '''
        check if the new point can directly reach the destination
        :param newPoint:
        :return: True or False
        '''
        return Tools.is_legal_point(newPoint, self.destination,
                                    self.obstacles, self.safeRadius)

    def get_legal_children_point(self, tree):
        '''
        recursive function util find a legal new point
        :return: null, if a legal new point was fond, it will be passed to self.newPoint
        '''
        leadPoint = self.lead_point()
        self.find_children_point_from_tree(tree, leadPoint, self.stepSize)
        isLegal = Tools.is_legal_point((self.closestNode.locationX, self.closestNode.locationY), self.newPoint_closest,
                                       self.obstacles, self.safeRadius)

        if isLegal is not True:
            self.get_legal_children_point(tree)

    def find_children_point_from_tree(self, tree, leadPoint, stepSize):
        """
        find the closest node to a leadPoint from the tree, then get a next child location
        :param tree:
        :param leadPoint:
        :return: a child location(x, y)
        """
        self.minDistance = 0
        self.find_closest_node(tree, leadPoint)
        self.newPoint_closest = self.find_children_point_from_closestnode(self.closestNode, leadPoint, stepSize)

    def find_children_point_from_closestnode(self, closestNode, leadPoint, stepSize):
        '''
        get a next child location
        :return: a child location(x, y)
        '''
        vectorStart = (closestNode.locationX, closestNode.locationY)
        vectorEnd = leadPoint

        return Tools.find_point_on_vector(vectorStart, vectorEnd, stepSize)

    def find_closest_node(self, root, leadPoint):
        """
        recursive function, find a closest node to the lead point.
        :param root:
        :param leadPoint:
        :return:
        """
        distance = Tools.getDistance((root.locationX, root.locationY), leadPoint)
        if self.minDistance == 0 or self.minDistance > distance:  # find the closest node
            self.minDistance = distance
            self.closestNode = root

        for child in root.children:
            self.find_closest_node(child, leadPoint)

    def calculate_search_space(self):
        """
        calculate the search space
        :param self:
        :return: search space = {
                                    'max_left':
                                    'min_right':
                                    'min_top':
                                    'max_down':
                                }
        """
        return Tools.get_search_space(self.obstacles, self.departure, self.destination, self.safeRadius)

    def lead_point(self):
        """
        a lead point determin the direction of a growing new node.
        :param self:
        :return: a lead point
        """
        x = np.random.uniform(self.searchSpace['min_right'], self.searchSpace['max_left'])
        y = np.random.uniform(self.searchSpace['max_down'], self.searchSpace['min_top'])
        return (x, y)
