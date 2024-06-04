class Edge:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2
        self.weight = Edge.__euclidean_distance(node1, node2)

    @staticmethod
    def __euclidean_distance(node1, node2):
        return ((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2) ** 0.5
