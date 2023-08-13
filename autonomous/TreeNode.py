class TreeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX  # location X
        self.locationY = locationY  # location Y
        self.children = []  # children list
        self.parent = None  # parent node
        self.parents = []
        self.bezier = []  # Bezier curve connecting the previous node and the next node
