import math

from reactor.geometry.vector import Vector2
from roomedge import RoomEdge


class Room:
    
    def __init__(self):
        self.template_type = -1
        self.flag_fixed = False
        self.boundary_type = 0
        self.vertices = []
        self.walls = []
        self.door_flags = []
        self.centre_shift = Vector2(0, 0)
        self.energy = 1
        self.reset_energy()
        self.reset_door_flags()

    @property
    def num_vertices(self):
        return len(self.vertices)

    @property
    def num_edges(self):
        return len(self.vertices)

    @property
    def num_walls(self):
        return len(self.walls)

    def __str__(self):
        str_ = f'Room [{id(self)}] with {self.num_vertices} vertices\n'
        for i, vertex in enumerate(self.vertices):
            str_ += f'    {i}th vertex: {vertex}\n'
        return str_

    def get_edge(self, idx):
        idx1 = idx
        idx2 = (idx + 1) % self.num_vertices
        edge = RoomEdge(self.vertices[idx1], self.vertices[idx2]) 
        edge.idx1 = idx1
        edge.idx2 = idx2
        #print(self.num_of_edges, self.door_flags)
        edge.door_flag = self.door_flags[idx]
        return edge
    
    def get_room_centre(self):
        centre = Vector2(0, 0)
        if not self.num_vertices:
            return centre

        # TODO: min / max can take a list.
        pos_min = Vector2(1e10, 1e10)
        pos_max = Vector2(-1e10, -1e10)
        for i in range(self.num_vertices):
            pi = self.vertices[i]
            for j in range(2):
                pos_min[j] = min(pos_min[j], pi[j])
                pos_max[j] = max(pos_max[j], pi[j])
    
        return (pos_min + pos_max) * 0.5
    
    def get_shifted_room_centre(self):
        return self.get_room_centre() + self.centre_shift
    
    def translate_room(self, trans):
        for i in range(self.num_vertices):
            self.vertices[i] += trans
    
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
        centre = self.get_room_centre()
        for i in range(self.num_vertices):
            pi = self.vertices[i] - centre
            self.vertices[i] = centre + pi * scaling
        self.centre_shift = self.centre_shift * scaling
    
    def get_room_bounding_box(self):
        p_min = Vector2(1e10, 1e10)
        p_max = Vector2(-1e10, -1e10)
        for i in range(self.num_vertices):
            pi = self.vertices[i]
            for j in range(2):
                p_min[j] = min(p_min[j], pi[j])
                p_max[j] = max(p_max[j], pi[j])
        return p_min, p_max
    
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
