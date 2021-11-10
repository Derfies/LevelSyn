import math

from reactor.geometry.vector import Vector2, Vector3
from reactor.geometry.orthogonalpolygon import OrthogonalPolygon, Edge
from reactor.const import POSITION

from linebase import LineBase


class RoomMixin:

    # TODO: Reintegrate LineBase class.

    def __str__(self):
        return f'RoomEdge: {self.pos1} {self.pos2}'

    @property
    def length(self):
        return (self.pos2 - self.pos1).mag()

    @property
    def sq_length(self):
        return (self.pos2 - self.pos1).mag2()

    @property
    def door_flag(self):
        return self._g.edges[self._edge]['door_flag']

    @door_flag.setter
    def door_flag(self, value):
        self._g.edges[self._edge]['door_flag'] = value

    @property
    def direction(self):
        return self.pos2 - self.pos1

    @property
    def direction3d(self):
        direction = self.direction
        return Vector3(direction[0], direction[1], 0)


class RoomEdge(RoomMixin, Edge):

    # TODO: Core code still assumes standalone room edge class.... :S
    pass


class RoomEdge2(RoomMixin):

    def __init__(self, pos1, pos2):
        self.pos1 = pos1
        self.pos2 = pos2



class Room(OrthogonalPolygon):

    edge_cls = RoomEdge
    
    def __init__(self):
        super().__init__()
        self.template_type = None
        self.flag_fixed = False
        self.boundary_type = 0
        self.walls = []
        self.centre_shift = Vector2(0, 0)
        self.energy = 1
        self.reset_energy()
        self.reset_door_flags()

    @property
    def num_vertices(self):
        return self.number_of_nodes()

    @property
    def num_edges(self):
        return self.number_of_edges()

    @property
    def num_walls(self):
        return len(self.walls)

    def __str__(self):
        str_ = f'Room [{id(self)}] with {self.num_vertices} vertices\n'
        for i, vertex in enumerate(self.vertices):
            str_ += f'    {i}th vertex: {vertex}\n'
        return str_

    def get_bounding_box(self):
        # p_min = Vector2(1e10, 1e10)
        # p_max = Vector2(-1e10, -1e10)
        # for i in range(self.num_vertices):
        #     pi = self.vertices[i]
        #     for j in range(2):
        #         p_min[j] = min(p_min[j], pi[j])
        #         p_max[j] = max(p_max[j], pi[j])
        # return p_min, p_max
        xs, ys = [], []
        for node in self.get_nodes():
            xs.append(node.position[0])
            ys.append(node.position[1])
        min_pos = Vector2(min(xs), min(ys))
        max_pos = Vector2(max(xs), max(ys))
        return min_pos, max_pos
    
    def get_centre(self):
        # xs, ys = [], []
        # for node in self.get_nodes():
        #     xs.append(node.position[0])
        #     ys.append(node.position[1])
        # min_pos = Vector2(min(xs), min(ys))
        # max_pos = Vector2(max(xs), max(ys))
        # return (min_pos + max_pos) * 0.5
        min_pos, max_pos = self.get_bounding_box()
        return (min_pos + max_pos) * 0.5
    
    def get_shifted_room_centre(self):
        return self.get_centre() + self.centre_shift
    
    def translate(self, trans):
        for node in self.nodes:
            self.nodes[node][POSITION] += trans
    
    def rotate_room(self, rad):
        cv = math.cos(rad)
        sv = math.sin(rad)
        for i in range(self.num_vertices):
            p0 = self.vertices[i][0]
            p1 = self.vertices[i][1]
            self.vertices[i][0] = p0 * cv + p1 * sv
            self.vertices[i][1] = -p0 * sv + p1 * cv
    
        p0 = self.centre_shift[0]
        p1 = self.centre_shift[1]
        self.centre_shift[0] = p0 * cv + p1 * sv
        self.centre_shift[1] = -p0 * sv + p1 * cv
    
    def scale_room(self, scaling):
        centre = self.get_centre()
        for i in range(self.num_vertices):
            pi = self.vertices[i] - centre
            self.vertices[i] = centre + pi * scaling
        self.centre_shift = self.centre_shift * scaling
    
    # def get_room_bounding_box(self, bounding_box):
    #     pos_min, pos_max = Vector2(0, 0), Vector2(0, 0)
    #     self.get_room_bounding_box(pos_min, pos_max)
    #     bounding_box.pos_min = pos_min
    #     bounding_box.pos_max = pos_max
    
    def init_walls(self):
        self.walls.clear()
        for i in range(self.num_vertices):
            idx1 = i
            idx2 = (i + 1) % self.num_vertices
            pos1 = self.vertices(idx1)
            pos2 = self.vertices(idx2)
            self.walls.append(RoomWall(pos1, pos2))
    
    def erase_wall(self, idx):
        if idx >= self.num_walls:
            return False
        self.walls.erase(self.walls.begin() + idx)
        return True
    
    def reset_door_flags(self):
        self.door_flags = [False] * self.num_edges
    
    def set_door_flag(self, edge_idx, door_flag):
        self.door_flags[edge_idx] = door_flag
    
    def get_door_flag(self, edge_idx):
        return self.door_flags[edge_idx]
    
    def has_restricted_door_position(self):
        for i in range(len(self.door_flags)):
            if not self.door_flags[i]:
                return True
        return False

    def update_energy(self, factor):
        self.energy *= factor

    def reset_energy(self):
        self.energy = 1
