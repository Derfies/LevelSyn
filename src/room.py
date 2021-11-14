from reactor.const import POSITION
from reactor.geometry.vector import Vector2, Vector3
from linebase import LineBase


class RoomNode:

    def __init__(self, room, node):
        self._room = room
        self._node = node

    @property
    def position(self):
        return self._room.origin + self._room.g.nodes[self._node][POSITION]


class RoomEdge(LineBase):

    def __init__(self, room, edge):
        self._room = room
        self._edge = edge

    def __str__(self):
        return f'RoomEdge: {self.pos1} {self.pos2}'

    @property
    def pos1(self):
        return self._room.origin + self._room.g.nodes[self._edge[0]][POSITION]

    @property
    def pos2(self):
        return self._room.origin + self._room.g.nodes[self._edge[1]][POSITION]

    @property
    def door_flag(self):
        return self._room.g.edges[self._edge]['door_flag']

    @door_flag.setter
    def door_flag(self, value):
        self._room.g.edges[self._edge]['door_flag'] = value


class Room:

    node_cls = RoomNode
    edge_cls = RoomEdge

    def __init__(self, g):
        self.g = g
        self.type = None
        self.origin = Vector2(0, 0)
        self._bounding_box = None

    def __copy__(self):
        cls = self.__class__
        copy = cls(self.g)
        copy.type = self.type
        copy.origin = Vector2(*self.origin)
        copy._bounding_box = self._bounding_box
        return copy

    def get_node(self, node):
        return self.__class__.node_cls(self, node)

    def get_nodes(self):
        for node in self.g.nodes:
            yield self.get_node(node)

    def get_edge(self, edge):
        return self.__class__.edge_cls(self, edge)

    def get_edges(self):
        for edge in self.g.edges:
            yield self.get_edge(edge)

    def __str__(self):
        str_ = f'Room [{id(self)}] with {self.g.number_of_nodes} vertices\n'
        for i, node in enumerate(self.g.nodes):
            str_ += f'    {i}th node: {node}\n'
        return str_

    @property
    def bounding_box(self):
        if self._bounding_box is None:
            xs, ys = [], []
            for node in self.get_nodes():
                xs.append(node.position[0])
                ys.append(node.position[1])
            min_pos = Vector2(min(xs), min(ys))
            max_pos = Vector2(max(xs), max(ys))
            self._bounding_box = min_pos, max_pos
        min_pos, max_pos = self._bounding_box
        return min_pos + self.origin, max_pos + self.origin

    @property
    def centre(self):
        min_pos, max_pos = self.bounding_box
        return (min_pos + max_pos) * 0.5
    
    def translate(self, trans):
        self.origin += trans
