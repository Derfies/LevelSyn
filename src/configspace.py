import copy
import math
import random

import networkx as nx

from reactor.geometry.vector import Vector2, Vector3, Vector4
from reactor.geometry.orthogonalpolygon import OrthogonalPolygon
from reactor import utils

from levelconfig import LevelConfig
from clipperwrapper import compute_collide_area
from levelmath import (
    NUMERICAL_TOLERANCE,
    NUMERICAL_TOLERANCE_SQ,
    point_to_line_sq_distance,
    point_to_segment_sq_distance,
    room_contact,
    segment_intersection,
    sort_vec_pr,
)
from linebase import LineBase
from room import Room
from roomedge import RoomEdge
from roomtemplates import RoomTemplates


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

        # HAX. Not sure how the c++ gets around this.
        # TODO: Separate constructors into functions.
        if room1 is None and room2 is None:
            return

        if self.flag_precomputed and room1.template_type >= 0 and room2.template_type >= 0:
            cs = self.precomputed_table[room1.template_type][room2.template_type]
            cs.translate_config_space(room1.room_centre)
            
            # TODO: Looks like another constructor / class method. Move to
            # __new__
            #*self = cs
            return

        # Why does 0.5 makes everything better?
        levelconfig = LevelConfig()
        contact_thresh = levelconfig.ROOM_CONTACT_THRESHOLD * 0.5
        for i in range(room1.num_edges):
            edge1 = room1.get_edge(i)
            for j in range(room2.num_edges):
                edge2 = room2.get_edge(j)

                # Ignore edge combinations where there is no door.
                if not edge1.door_flag or not edge2.door_flag:
                    continue

                # Ignore edges that aren't parallel.
                cross = edge1.direction3d.cross(edge2.direction3d)
                if cross.mag2() > NUMERICAL_TOLERANCE:
                    continue

                # TODO: Skip edges that are the same direction...?
                # Check the sign of the dot of both edge directions.
                # Seems like this is normally taken care of below, during coll
                # tests.
                dir1 = edge1.direction
                dir2 = edge2.direction
                dot = dir1.dot(dir2)
                if math.copysign(1, dot) > 0:
                    continue

                # Build a list of contact sample points.
                vec_pr = [
                    edge1.pos1 - edge2.pos1,
                    edge1.pos1 - edge2.pos2,
                    edge1.pos2 - edge2.pos1,
                    edge1.pos2 - edge2.pos2
                ]

    #ifdef ACCURATE_CONFIG_SPACE

                dir1 = dir1.normalise()
                shift = dir1 * contact_thresh
                for k in range(4):
                    vec_pr.extend((
                        vec_pr[k] + shift,
                        vec_pr[k] - shift,
                    ))
    #endif
                #print('')
                #print('before:')
                #for d in vec_pr:
                #    print('    ->', d)
                sort_vec_pr(vec_pr)
                #print('after:')
                #for d in vec_pr:
                #    print('    ->', d)
                #print('')
                for k in range(len(vec_pr)):
                    pr1 = vec_pr[k]
                    pr2 = vec_pr[k - 1]

                    #print('->', pr1, pr2)

                    # Ignore if the two points are coincident.
                    if (pr2 - pr1).mag2() < NUMERICAL_TOLERANCE:
                        continue
                    pr3 = (pr1 + pr2) * 0.5

                    # for sample_point in (pr1, pr2, pr3):
                    #     sample_room = copy.deepcopy(room2)
                    #     sample_room.translate_room(sample_point)
                    #
                    #     # TODO: Could replace with touches / intersects?
                    #     # Need to make rooms derive from orthogonal poly first.
                    #     if compute_collide_area(room1, sample_room):#> NUMERICAL_TOLERANCE:
                    #         print('FOUND OVERLAP')
                    #         continue
                    #     if room_contact(room1, sample_room) < contact_thresh - NUMERICAL_TOLERANCE:
                    #         print('FOUND contact')
                    #         continue
                    room2n1 = copy.deepcopy(room2)
                    room2n1.translate_room(pr1)

                    # TODO: Could replace with touches / intersects?
                    if compute_collide_area(room1, room2n1):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n1) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif
                    room2n2 = copy.deepcopy(room2)
                    room2n2.translate_room(pr2)
                    if compute_collide_area(room1, room2n2):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n2) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif
                    room2n3 = copy.deepcopy(room2)
                    room2n3.translate_room(pr3)
                    if compute_collide_area(room1, room2n3):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n3) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif

                    pos1 = room2.get_room_centre() + pr1
                    pos2 = room2.get_room_centre() + pr2
                    line = ConfigLine(pos1, pos2)
                    #print('adding:', line)
                    self.config_lines.append(line)

        # for line in self.config_lines:
        #     print('line:', line.pos1.x, line.pos1.y, line.pos2.x, line.pos2.y)

        self.merge()

    @property
    def num_lines(self):
        return len(self.config_lines)

    def __str__(self):
        str_ = f'ConfigSpace with {self.num_lines} lines\n'
        for i in range(self.num_lines):
            str_ += f'    {i}th line: {self.config_lines[i]}\n'
        return str_

    def randomly_sample_config_space(self):
        r = random.random()
        return self.randomly_sample_config_space_continuous() if r >= 0.5 else self.randomly_sample_config_space_discrete()

    def randomly_sample_config_space_continuous(self):
        line_index = random.randint(0, self.num_lines - 1)
        return self.config_lines[line_index].randomly_sample_config_line()

    def randomly_sample_config_space_discrete(self):
        line_index = random.randint(0, self.num_lines - 1)
        return self.config_lines[line_index].randomly_sample_config_line_discrete()

    def smartly_sample_config_space(self):
        positions = []
        for config_line in self.config_lines:
            r = random.random()
            pos = config_line.randomly_sample_config_line() if r >= 0.5 else config_line.randomly_sample_config_line_discrete()
            positions.append(pos)
        return positions

    @classmethod
    def find_intersection(cls, config_space1, config_space2):
        intersect_space = cls()
        for i1 in range(config_space1.num_lines):
            config_line1 = config_space1.config_lines[i1]
            for i2 in range(config_space2.num_lines):
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

        # TODO: Not sure about this one...
        config_space_new = copy.deepcopy(config_space)
        if not config_space.config_lines:
            config_space_new.config_lines.append(config_line)
            return config_space_new

        merge_flag = False
        for i in range(config_space.num_lines):
            line = config_space_new.config_lines[i]
            edge1 = RoomEdge(line.pos1, line.pos2)
            edge2 = RoomEdge(config_line.pos1, config_line.pos2)
            sq_length1 = edge1.sq_length
            sq_length2 = edge2.sq_length

            if sq_length1 >= NUMERICAL_TOLERANCE and sq_length2 >= NUMERICAL_TOLERANCE:
                cross = edge1.direction3d.cross(edge2.direction3d)
                if cross.mag2() > NUMERICAL_TOLERANCE:
                    continue

            elif sq_length1 < NUMERICAL_TOLERANCE and sq_length2 > NUMERICAL_TOLERANCE:
                continue

            elif sq_length1 < NUMERICAL_TOLERANCE and sq_length2 < NUMERICAL_TOLERANCE:
                if (edge1.pos1 - edge2.pos1).mag2() < NUMERICAL_TOLERANCE:
                    merge_flag = True
                    break
                continue

            # Getting duplicate config lines here due to some bug which isn't
            # detecting colinear lines.
            # Switched these args around for the moment. Don't think that's the
            # best solution.
            #if point_to_segment_sq_distance(edge2.pos1, edge1) < NUMERICAL_TOLERANCE_SQ or point_to_segment_sq_distance(edge2.pos2, edge1) < NUMERICAL_TOLERANCE_SQ:
            #if point_to_segment_sq_distance(edge1.pos1, edge2) < NUMERICAL_TOLERANCE_SQ or point_to_segment_sq_distance(edge1.pos2, edge2) < NUMERICAL_TOLERANCE_SQ:
            this = point_to_segment_sq_distance(edge1.pos1, edge2) < NUMERICAL_TOLERANCE_SQ or point_to_segment_sq_distance(edge1.pos2, edge2) < NUMERICAL_TOLERANCE_SQ
            that = point_to_segment_sq_distance(edge2.pos1, edge1) < NUMERICAL_TOLERANCE_SQ or point_to_segment_sq_distance(edge2.pos2, edge1) < NUMERICAL_TOLERANCE_SQ
            if this or that:
                pos_min1 = edge1.pos1.minimum(edge1.pos2)
                pos_max1 = edge1.pos1.maximum(edge1.pos2)
                pos_min2 = edge2.pos1.minimum(edge2.pos2)
                pos_max2 = edge2.pos1.maximum(edge2.pos2)
                pos_min = pos_min1.minimum(pos_min2)
                pos_max = pos_max1.maximum(pos_max2)
                pos1, pos2 = Vector2(0, 0), Vector2(0, 0)
                for j in range(2):
                    pos1[j] = pos_min[j] if line.pos1[j] == pos_min1[j] else pos_max[j]
                    pos2[j] = pos_max[j] if line.pos1[j] == pos_min1[j] else pos_min[j]

                line.pos1 = pos1
                line.pos2 = pos2
                merge_flag = True
                break

        if not merge_flag:
            config_space_new.config_lines.append(config_line)

        return config_space_new

    def merge(self):

        # TODO: Reimplement this.
        #sort(self.config_lines.begin(), self.config_lines.end(), self.compare_config_line_length)
        config_space_new = ConfigSpace()
        for config_line in self.config_lines:
            config_space_new = ConfigSpace.find_union(config_space_new, config_line)
        self.config_lines = config_space_new.config_lines

    def get_config_space_size(self):
        sz = 0
        for config_line in self.config_lines:
            sz += config_line.config_lines_length()
        return sz

    def translate_config_space(self, trans):
        for config_line in self.config_lines:
            config_line.translate_config_line(trans)

    @staticmethod
    def compare_config_line_length(line1, line2):
        return line1.config_lines_sq_length() > line2.config_lines_sq_length()

    def precompute_table(self, rooms):
        self.flag_precomputed = False
        self.precomputed_table.clear()
        num_rooms = len(rooms)
        for i in range(num_rooms):
            config_spaces = []
            room1 = rooms[i]
            room1.translate_room(-room1.get_room_centre())
            for j in range(num_rooms):
                config_spaces.append(ConfigSpace(room1, rooms[j]))
            self.precomputed_table.append(config_spaces)
        self.flag_precomputed = True


if __name__ == '__main__':

    '''
    #
    # for l in [
    #     # LineBase(Vector2(1.5, 0), Vector2(1.5, 4)),
    #     # LineBase(Vector2(1.5, 4), Vector2(1.5, 0)),
    #     # LineBase(Vector2(1.5, 2), Vector2(1.5, 0)),
    #     LineBase(Vector2(1.5, 4), Vector2(1.5, -2)),
    # ]:
    #     print('')
    #     for p in [
    #         #Vector2(1.5, -4),
    #         # Vector2(1.5, -2),
    #         # Vector2(1.5, 0),
    #         Vector2(1.5, 2),
    #         Vector2(1.5, 4),
    #         # Vector2(1.5, 6),
    #     ]:
    #         print(f'point_to_segment_sq_distance: {l.pos1.x},{l.pos1.y} -> {l.pos2.x},{l.pos2.y}', ':', f'{p.x},{p.y}', '=', point_to_segment_sq_distance(p, l))

    #print('point_to_segment_sq_distance:', p2, l.pos1, l.pos2, point_to_segment_sq_distance(p2, l))
    room1 = Room()
    room1.vertices.append(Vector2(0, 0))
    room1.vertices.append(Vector2(0, 2))
    room1.vertices.append(Vector2(2, 2))
    room1.vertices.append(Vector2(2, 4))
    room1.vertices.append(Vector2(4, 4))
    room1.vertices.append(Vector2(4, 2))
    room1.vertices.append(Vector2(6, 2))
    room1.vertices.append(Vector2(6, 0))
    # room1.vertices.append(Vector2(1, 2))
    # room1.vertices.append(Vector2(1, 0))
    room1.reset_door_flags()
    room1.set_door_flag(2, True)
    for i in range(len(room1.vertices)):
        room1.set_door_flag(i, True)
    print(room1)

    room2 = Room()
    room2.vertices.append(Vector2(12, 0))
    room2.vertices.append(Vector2(12, 1))
    room2.vertices.append(Vector2(13, 1))
    room2.vertices.append(Vector2(13, 0))
    room2.reset_door_flags()
    room2.set_door_flag(0, True)
    # for i in range(len(room2.vertices)):
    #     room2.set_door_flag(i, True)

    # room2 = Room()
    # room2.vertices.append(Vector2(2, 0))
    # room2.vertices.append(Vector2(2, 2))
    # room2.vertices.append(Vector2(3, 2))
    # room2.vertices.append(Vector2(3, 0))
    # room2.reset_door_flags()
    # room2.set_door_flag(0, True)

    print(room2)

    config_space = ConfigSpace(room1, room2)
    print(config_space)

    r1 = OrthogonalPolygon.from_positions(room1.vertices)
    r2 = OrthogonalPolygon.from_positions(room2.vertices)

    # print(room1)
    # print(room2)

    g = nx.Graph()
    g = nx.compose(g, r1)
    g = nx.compose(g, r2)
    for line in config_space.config_lines:
        p = OrthogonalPolygon.from_positions([line.pos1, line.pos2])
        g = nx.compose(g, p)
    utils.draw_map(g, [])

    '''
    roomtemplates = RoomTemplates()
    roomtemplates.load(r'C:\Users\Jamie Davies\Documents\git\LevelSyn\data\building_blocks_fig1.xml')
    g = nx.Graph()
    for i, room in enumerate(roomtemplates.rooms):
        vertices = []
        for v in room.vertices:
            vertices.append(v + Vector2(i, 0))
        p = OrthogonalPolygon.from_positions(vertices)
        g = nx.compose(g, p)

    cs = ConfigSpace()
    cs.precompute_table(roomtemplates.rooms)

    import os
    from reactor.geometry.orthogonalpolygon import OrthogonalPolygon

    m = 0
    dir_name = r'C:\Users\Jamie Davies\Documents\git\reactor\output\debug'


    for i, spaces in enumerate(roomtemplates.rooms):

        for j, space in enumerate(cs.precomputed_table[i]):

            g = nx.Graph()
            r1 = OrthogonalPolygon.from_positions(list(reversed(roomtemplates.rooms[i].vertices)))
            g = nx.compose(g, r1)

            vertices = []
            for v in roomtemplates.rooms[j].vertices:
                vertices.append(v + Vector2(1, 0))
            r2 = OrthogonalPolygon.from_positions(list(reversed(vertices)))
            #for node in r2:
                # print('before:', r2.nodes[node]['position'])
                # r2.nodes[node]['position'] += Vector2(1, 0)
                # print('after:', r2.nodes[node]['position'])
            g = nx.compose(g, r2)

            for line in space.config_lines:
                p = OrthogonalPolygon.from_positions([line.pos1, line.pos2])
                # for node in p:
                #     p.node['position'] += Vector2(2, 0)
                g = nx.compose(g, p)

            file_name = '{0:03d}'.format(m)
            file_path = os.path.join('output', dir_name, file_name) + '.png'
            #p = OrthogonalPolygon.from_positions(vertices)
            #g = nx.compose(g, p)
            #for node in g:

            utils.draw_graph(g, file_path)

            m += 1

    #utils.draw_map(g, [])