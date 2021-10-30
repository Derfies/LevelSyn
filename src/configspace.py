import random

from reactor.geometry.vector import Vector2, Vector3, Vector4

import levelconfig
from levelmath import (
    NUMERICAL_TOLERANCE,
    NUMERICAL_TOLERANCE_SQ,
    point_to_line_sq_distance,
    point_to_segment_sq_distance,
    room_contact,
    segment_intersection,
)
from linebase import LineBase
from room import Room
from roomedge import RoomEdge


class ConfigLine(LineBase):
    
    def __str__(self):
        return f'p1: {self.pos1} p2: {self.pos2}'
    
    def randomly_sample_config_line(self):
        wt1 = random.random()
        wt2 = 1 - wt1
        pos = wt1 * self.pos1 + wt2 * self.pos2
        return pos
    
    def randomly_sample_config_line_discrete(self):
        r = random.random()
        pos = self.pos1 if r >= 0.5 else self.pos2
        return pos
    
    def config_lines_length(self):
        return (self.pos1 - self.pos2).mag()
    
    def config_lines_sq_length(self):
        return (self.pos1 - self.pos2).mag2()
    
    def translate_config_line(self, trans):
        self.pos1 += trans
        self.pos2 += trans


class ConfigSpace:

    def __init__(self, room1=None, room2=None):
        self.config_lines = []
        self.precomputed_table = []
        self.flag_precomputed = False

        if room1 is None and room2 is None:
            return

        if self.flag_precomputed and room1.template_type >= 0 and room2.template_type >= 0:
            cs = self.precomputed_table[room1.template_type][room2.template_type]
            cs.translate_config_space(room1.room_centre)
            
            # TODO: Looks like another constructor / class method
            #*self = cs
            return

        print(room1, room2)
        print(room1.num_of_edges, room2.num_of_edges)

        # Why does 0.5 makes everything better?
        contact_thresh = levelconfig.ROOM_CONTACT_THRESH * 0.5
        for i in range(room1.num_of_edges):
            edge1 = room1.get_edge(i)
            for j in range(room2.num_of_edges):
                edge2 = room2.get_edge(j)
                if not edge1.door_flag or not edge2.door_flag:
                    continue

                cp = edge1.direction3d.cross(edge2.direction3d)
                if cp.mag2() > NUMERICAL_TOLERANCE:
                    continue

                vec_pr = Vector4(0, 0, 0, 0)
                vec_pr[0] = edge1.pos1 - edge2.pos1
                vec_pr[1] = edge1.pos1 - edge2.pos2
                vec_pr[2] = edge1.pos2 - edge2.pos1
                vec_pr[3] = edge1.pos2 - edge2.pos2
    #ifdef ACCURATE_CONFIG_SPACE
                dir_ = edge1.direction
                dir_ = dir_.normalize()
                shift = dir_ * contact_thresh
                for k in range(4):
                    vec_pr.push_back(vec_pr[k] + shift)
                    vec_pr.push_back(vec_pr[k] - shift)
    #endif
                SortVecPr(vec_pr)
                for k in range(len(vec_pr)):
                    pr1 = vec_pr[k]
                    pr2 = vec_pr[k - 1]
                    if (pr2 - pr1).mag2() < NUMERICAL_TOLERANCE:
                        continue

                    pr3 = (pr1 + pr2) * 0.5
                    room2n1 = room2
                    room2n1.translate_room(pr1)
                    if wrapper.ComputeCollideArea(room1, room2n1) > NUMERICAL_TOLERANCE:
                        continue
    #ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n1) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
    #endif
                    room2n2 = room2
                    room2n2.translate_room(pr2)
                    if wrapper.ComputeCollideArea(room1, room2n2) > NUMERICAL_TOLERANCE:
                        continue
    #ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n2) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
    #endif
                    room2n3 = room2
                    room2n3.translate_room(pr3)
                    if wrapper.ComputeCollideArea(room1, room2n3) > NUMERICAL_TOLERANCE:
                        continue
    #ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n3) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
    #endif
                    pos1 = room2.room_centre + pr1
                    pos2 = room2.room_centre + pr2
                    line = ConfigLine(pos1, pos2)
                    self.add_config_line(line)

        self.merge()

    @property
    def num_of_lines(self):
        return len(self.config_lines)

    def __str__(self):
        str_ = ''
        for i in range(self.num_of_lines):
            str_ += 'The {i}th line:\n'
            str_ += str(self.config_lines[i])
        return str_

    def randomly_sample_config_space(self):
        r = random.random()
        return self.randomly_sample_config_space_continuous() if r >= 0.5 else self.randomly_sample_config_space_discrete()

    def randomly_sample_config_space_continuous(self):
        line_index = random.randint(0, self.num_of_lines - 1)
        return self.config_lines[line_index].randomly_sample_config_line()

    def randomly_sample_config_space_discrete(self):
        line_index = random.randint(0, self.num_of_lines - 1)
        return self.config_lines[line_index].randomly_sample_config_line_discrete()

    def smartly_sample_config_space(self):
        vec_pos = []
        for i in range(self.num_of_lines):
            config_line = self.config_line[i]
            r = random.random()
            pos = config_line.randomly_sample_config_line() if r >= 0.5 else config_line.randomly_sample_config_line_discrete()
            vec_pos[i] = pos
        return vec_pos

    @classmethod
    def find_intersection(cls, config_space1, config_space2):
        intersect_space = cls()
        for i1 in range(config_space1.num_of_lines):
            config_line1 = config_space1.config_lines[i1]
            for i2 in range(config_space2.num_of_lines):
                config_line2 = config_space2.config_lines[i2]
                
                if config_line1.config_lines_sq_length() < NUMERICAL_TOLERANCE_SQ and config_line2.config_lines_sq_length() < NUMERICAL_TOLERANCE_SQ:
                    if (config_line1.pos1 - config_line2.pos1).mag2() < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.add_config_line(config_line1)
                    continue
                    
                elif config_line1.config_lines_sq_length() < NUMERICAL_TOLERANCE_SQ:
                    edge = RoomEdge(config_line2.pos1, config_line2.pos2)
                    if point_to_segment_sq_distance(config_line1.pos1, edge) < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.add_config_line(config_line1)
                    continue
                    
                elif config_line2.config_lines_sq_length() < NUMERICAL_TOLERANCE_SQ:
                    edge = RoomEdge(config_line1.pos1, config_line1.pos2)
                    if point_to_segment_sq_distance(config_line2.pos1, edge) < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.add_config_line(config_line2)
                    continue

                p11 = config_line1.pos1
                p12 = config_line1.pos2
                p21 = config_line2.pos1
                p22 = config_line2.pos2
                pe1 = Vector3(p12[0] - p11[0], p12[1] - p11[1], 0)
                pe2 = Vector3(p22[0] - p21[0], p22[1] - p21[1], 0)
                cp = pe1.cross(pe2)
                if cp.mag2() > NUMERICAL_TOLERANCE:
                    
                    # Not parallel...
                    flag_intersect, pi = segment_intersection(p11, p12, p21, p22)
                    if flag_intersect:
                        intersect_line = ConfigLine(pi)
                        intersect_space.add_config_line(intersect_line)

                else:

                    # Parallel...
                    pos_min1 = min_union(p11, p12)
                    pos_max1 = max_union(p11, p12)
                    pos_min2 = min_union(p21, p22)
                    pos_max2 = max_union(p21, p22)
                    flag_overlap = True
                    for j in range(2):
                        if pos_max1[j] < pos_min2[j] - NUMERICAL_TOLERANCE or pos_min1[j] > pos_max2[j] + NUMERICAL_TOLERANCE:
                            flag_overlap = False
                            break

                    if not flag_overlap:
                        continue

                    d1 = point_to_line_sq_distance(p21, p12, p11)
                    d2 = point_to_line_sq_distance(p22, p12, p11)
                    if d1 > NUMERICAL_TOLERANCE_SQ or d2 > NUMERICAL_TOLERANCE_SQ:
                        flag_overlap = False

                    if not flag_overlap:
                        continue

                    p1, p2 = Vector2(0, 0), Vector2(0, 0)
                    for d in range(2):
                        p1[d] = max(min(p11[d], p12[d]), min(p21[d], p22[d]))
                        p2[d] = min(max(p11[d], p12[d]), max(p21[d], p22[d]))

                    intersect_line = ConfigLine(p1, p2)
                    intersect_space.add_config_line(intersect_line)

        return intersect_space

    @classmethod
    def find_union(cls, config_space, config_line):
        config_space_new = config_space
        if not config_space.config_lines:
            config_space_new.add_config_line(config_line)
            return config_space_new

        merge_flag = False
        for i in range(config_space.num_of_lines):
            line = config_space_new.self.config_lines[i]
            edge1 = RoomEdge(line.pos1, line.pos2)
            edge2 = RoomEdge(config_line.pos1, config_line.pos2)
            sqlength1 = edge1.sqlength
            sqlength2 = edge2.sqlength
            if sqlength1 >= NUMERICAL_TOLERANCE and sqlength2 >= NUMERICAL_TOLERANCE:
                cp = edge1.direction3d.cross(edge2.direction3d)
                if cp.mag2() > NUMERICAL_TOLERANCE:
                    continue

            elif sqlength1 < NUMERICAL_TOLERANCE and sqlength2 > NUMERICAL_TOLERANCE:
                continue

            elif sqlength1 < NUMERICAL_TOLERANCE and sqlength2 < NUMERICAL_TOLERANCE:
                if (edge1.pos1 - edge2.pos1).mag2() < NUMERICAL_TOLERANCE:
                    merge_flag = True
                    break
                continue

            if point_to_segment_sq_distance(edge1.pos1, edge2) < NUMERICAL_TOLERANCE_SQ or point_to_segment_sq_distance(edge1.pos2, edge2) < NUMERICAL_TOLERANCE_SQ:
                pos_min1 = min_union(edge1.pos1, edge1.pos2)
                pos_max1 = max_union(edge1.pos1, edge1.pos2)
                pos_min2 = min_union(edge2.pos1, edge2.pos2)
                pos_max2 = max_union(edge2.pos1, edge2.pos2)
                posMin = min_union(pos_min1, pos_min2)
                posMax = max_union(pos_max1, pos_max2)
                pos1, pos2 = Vector2(0, 0), Vector2(0, 0)
                for j in range(2):
                    pos1[j] = posMin[j] if line.pos1[j] == pos_min1[j] else posMax[j]
                    pos2[j] = posMax[j] if line.pos1[j] == pos_min1[j] else posMin[j]

                line.pos1 = pos1
                line.pos2 = pos2
                merge_flag = True
                break

        if not merge_flag:
            config_space_new.add_config_line(config_line)

        return config_space_new

    def merge(self):
        #sort(self.config_lines.begin(), self.config_lines.end(), self.compare_config_line_length)
        config_space_new = ConfigSpace()
        for i in range(self.num_of_lines):
            config_space_new = ConfigSpace.find_union(config_space_new, self.config_lines[i])
        self.config_lines = config_space_new.config_lines

    def get_config_space_size(self):
        sz = 0
        for i in range(self.num_of_lines):
            sz += self.config_lines[i].config_lines_length()
        return sz

    def translate_config_space(self, trans):
        for i in range(self.num_of_lines):
            self.config_lines[i].translate_config_line(trans)

    @staticmethod
    def compare_config_line_length(line1, line2):
        return line1.config_lines_sq_length() > line2.config_lines_sq_length()

    def precompute_table(self, rooms):
        self.flag_precomputed = False
        self.precomputed_table.clear()
        num_of_rooms = len(rooms)
        for i in range(num_of_rooms):
            config_spaces = []
            room1 = rooms[i]
            room1.translate_room(-room1.room_centre)
            for j in range(num_of_rooms):
                config_spaces.append(ConfigSpace(room1, rooms[j]))
            self.precomputed_table[i] = config_spaces
        self.flag_precomputed = True


if __name__ == '__main__':
    room1 = Room()
    room1.vertices.append(Vector2(0, 0))
    room1.vertices.append(Vector2(0, 1))
    room1.vertices.append(Vector2(1, 1))
    room1.vertices.append(Vector2(1, 0))
    room2 = Room()
    room2.vertices.append(Vector2(0, 0))
    room2.vertices.append(Vector2(0, 1))
    room2.vertices.append(Vector2(1, 1))
    room2.vertices.append(Vector2(1, 0))
    config_space = ConfigSpace(room1, room2)
    print(config_space)