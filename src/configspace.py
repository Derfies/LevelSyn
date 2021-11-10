import copy
import math
import random

import networkx as nx

from reactor.geometry.vector import Vector2, Vector3, Vector4
from reactor.geometry.orthogonalpolygon import OrthogonalPolygon
from reactor import utils

from simple_settings import settings
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
from room import RoomEdge, RoomEdge2
from roomtemplates import RoomTemplates


class ConfigLine(LineBase):
    
    def __str__(self):
        return f'p1: {self.pos1} p2: {self.pos2}'
    
    def randomly_sample(self):

        # TODO: Convert this to sample integers only.
        wt1 = random.random()
        wt2 = 1 - wt1
        pos = wt1 * self.pos1 + wt2 * self.pos2
        return pos
    
    def randomly_sample_discrete(self):
        r = random.random()
        return self.pos1 if r >= 0.5 else self.pos2

    def translate(self, trans):
        self.pos1 += trans
        self.pos2 += trans


class ConfigSpace:

    precomputed_table = []

    def __init__(self, room1=None, room2=None):
        self.lines = []

        if room1 is None and room2 is None:
            return

        if room1.template_type in self.precomputed_table and room2.template_type in self.precomputed_table[room1.template_type]:
            cs = self.precomputed_table[room1.template_type][room2.template_type]
            cs.translate_config_space(room1.room_centre)
            self.lines = copy.deepcopy(cs)
            return

        # Why does 0.5 makes everything better?
        contact_thresh = settings.ROOM_CONTACT_THRESHOLD * 0.5

        for edge1 in room1.get_edges():
            for edge2 in room2.get_edges():

                # Ignore edge combinations where there is no door.
                # TODO: Define doors with lines, not with bools.
                if not edge1.door_flag or not edge2.door_flag:
                    continue

                # Ignore edges that aren't parallel - a room cannot be joined
                # to another rooms except via parallel edges.
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
                sort_vec_pr(vec_pr)

                # Iterate this sample point and the next sample point along the
                # line.
                for k in range(len(vec_pr)):
                    pr1 = vec_pr[k]
                    pr2 = vec_pr[k - 1]

                    # Ignore if the two points are coincident.
                    if (pr2 - pr1).mag2() < NUMERICAL_TOLERANCE:
                        continue
                    pr3 = (pr1 + pr2) * 0.5

                    room2n1 = copy.deepcopy(room2)
                    room2n1.translate(pr1)

                    # TODO: Could replace with touches / intersects?
                    if compute_collide_area(room1, room2n1):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n1) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif
                    room2n2 = copy.deepcopy(room2)
                    room2n2.translate(pr2)
                    if compute_collide_area(room1, room2n2):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n2) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif
                    room2n3 = copy.deepcopy(room2)
                    room2n3.translate(pr3)
                    if compute_collide_area(room1, room2n3):# > NUMERICAL_TOLERANCE:
                        continue
                    # ifdef ACCURATE_CONFIG_SPACE
                    if room_contact(room1, room2n3) < contact_thresh - NUMERICAL_TOLERANCE:
                        continue
                    # endif

                    pos1 = room2.get_centre() + pr1
                    pos2 = room2.get_centre() + pr2
                    self.lines.append(ConfigLine(pos1, pos2))

        self.merge()

    @property
    def num_lines(self):
        return len(self.lines)

    def __str__(self):
        str_ = f'ConfigSpace with {self.num_lines} lines\n'
        for i in range(self.num_lines):
            str_ += f'    {i}th line: {self.lines[i]}\n'
        return str_

    def randomly_sample_config_space(self):
        r = random.random()
        return self.randomly_sample_config_space_continuous() if r >= 0.5 else self.randomly_sample_config_space_discrete()

    def randomly_sample_config_space_continuous(self):
        line_index = random.randint(0, self.num_lines - 1)
        return self.lines[line_index].randomly_sample()

    def randomly_sample_config_space_discrete(self):
        line_index = random.randint(0, self.num_lines - 1)
        return self.lines[line_index].randomly_sample_discrete()

    def smartly_sample_config_space(self):
        positions = []
        for config_line in self.lines:
            r = random.random()
            pos = config_line.randomly_sample() if r >= 0.5 else config_line.randomly_sample_discrete()
            positions.append(pos)
        return positions

    @classmethod
    def find_intersection(cls, config_space1, config_space2):
        intersect_space = cls()
        for config_line1 in config_space1.lines:
            for config_line2 in config_space2.lines:

                # Neither line has any length...
                if config_line1.sq_length < NUMERICAL_TOLERANCE_SQ and config_line2.sq_length < NUMERICAL_TOLERANCE_SQ:
                    if (config_line1.pos1 - config_line2.pos1).mag2() < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.lines.append(config_line1)
                    continue

                # 1st line has no length, test to see if it sits on the line
                elif config_line1.sq_length < NUMERICAL_TOLERANCE_SQ:
                    edge = RoomEdge2(config_line2.pos1, config_line2.pos2)
                    if point_to_segment_sq_distance(config_line1.pos1, edge) < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.lines.append(config_line1)
                    continue

                # 2nd line has no length, test to see if it sits on the line
                elif config_line2.sq_length < NUMERICAL_TOLERANCE_SQ:
                    edge = RoomEdge2(config_line1.pos1, config_line1.pos2)
                    if point_to_segment_sq_distance(config_line2.pos1, edge) < NUMERICAL_TOLERANCE_SQ:
                        intersect_space.lines.append(config_line2)
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
                    # The resulting intersection line is a single point where
                    # the lines intersect.
                    flag_intersect, pi = segment_intersection(p11, p12, p21, p22)
                    if flag_intersect:
                        intersect_line = ConfigLine(pi)
                        intersect_space.lines.append(intersect_line)

                else:

                    # Parallel...
                    pos_min1 = p11.minimum(p12)
                    pos_max1 = p11.maximum(p12)
                    pos_min2 = p21.minimum(p22)
                    pos_max2 = p21.maximum(p22)

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
                    intersect_space.lines.append(intersect_line)

        return intersect_space

    @classmethod
    def find_union(cls, config_space, config_line):

        # TODO: Not sure about this one...

        # If there are no lines, return a config space containing just the given
        # line.
        config_space_new = copy.deepcopy(config_space)
        if not config_space.lines:
            config_space_new.lines.append(config_line)
            return config_space_new

        merge_flag = False
        for i in range(config_space.num_lines):
            line = config_space_new.lines[i]
            edge1 = RoomEdge2(line.pos1, line.pos2)
            edge2 = RoomEdge2(config_line.pos1, config_line.pos2)
            sq_length1 = edge1.sq_length
            sq_length2 = edge2.sq_length

            # If the two edges have length...
            if sq_length1 >= NUMERICAL_TOLERANCE and sq_length2 >= NUMERICAL_TOLERANCE:

                # Only parallel lines can be merged.
                cross = edge1.direction3d.cross(edge2.direction3d)
                if cross.mag2() > NUMERICAL_TOLERANCE:
                    continue

            # Only a line with length can be merged.
            elif sq_length1 < NUMERICAL_TOLERANCE and sq_length2 > NUMERICAL_TOLERANCE:
                continue

            # If neither line has length, ignore unless..????
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

        # Merge the line in if one of the existing lines cannot be altered to
        # include its range.
        if not merge_flag:
            config_space_new.lines.append(config_line)

        return config_space_new

    def merge(self):

        # TODO: Reimplement this.
        #sort(self.config_lines.begin(), self.config_lines.end(), self.compare_config_line_length)
        config_space_new = ConfigSpace()
        for config_line in self.lines:
            config_space_new = ConfigSpace.find_union(config_space_new, config_line)
        self.lines = config_space_new.lines

    def get_config_space_size(self):
        return sum([line.length for line in self.lines])

    def translate_config_space(self, trans):
        for config_line in self.lines:
            config_line.translate(trans)

    @staticmethod
    def compare_config_line_length(line1, line2):
        return line1.sq_length > line2.sq_length

    @classmethod
    def precompute_table(cls, rooms):
        cls.precomputed_table.clear()
        for room in rooms:
            config_spaces = []
            room.translate(-room.get_centre())
            for other_room in rooms:
                config_spaces.append(ConfigSpace(room, other_room))
            cls.precomputed_table.append(config_spaces)


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
    roomtemplates.load(r'C:\Users\Jamie Davies\OneDrive\Documents\git\LevelSyn\data\building_blocks_fig1.xml')
    g = nx.Graph()
    for i, room in enumerate(roomtemplates.rooms):
        #vertices = []
        #for v in room.vertices:
        #    vertices.append(v + Vector2(i, 0))
        room.translate(Vector2(i, 0))
        #p = OrthogonalPolygon.from_positions(vertices)
        g = nx.compose(g, room)

    ConfigSpace.precompute_table(roomtemplates.rooms)
    cs = ConfigSpace()

    import os
    from reactor.geometry.orthogonalpolygon import OrthogonalPolygon

    m = 0
    dir_name = r'C:\Users\Jamie Davies\OneDrive\Documents\git\LevelSyn\debug'

    for i, spaces in enumerate(roomtemplates.rooms):
        for j, space in enumerate(cs.precomputed_table[i]):

            g = nx.Graph()
            vertices = []
            for node in roomtemplates.rooms[i].get_nodes():
                vertices.append(node.position)
            r1 = OrthogonalPolygon.from_positions(list(reversed(vertices)))
            g = nx.compose(g, r1)

            vertices = []
            # for v in roomtemplates.rooms[j].vertices:
            #     vertices.append(v + Vector2(0, 10))
            for node in roomtemplates.rooms[j].get_nodes():
                vertices.append(node.position + Vector2(0, 10))
            r2 = OrthogonalPolygon.from_positions(list(reversed(vertices)))
            #for node in r2:
                # print('before:', r2.nodes[node]['position'])
                # r2.nodes[node]['position'] += Vector2(1, 0)
                # print('after:', r2.nodes[node]['position'])
            g = nx.compose(g, r2)

            for line in space.lines:
                p = OrthogonalPolygon.from_positions([line.pos1, line.pos2])
                for node in p.nodes:
                    p.nodes[node]['position'] += Vector2(0, 0)
                g = nx.compose(g, p)

            file_name = '{0:03d}'.format(m)
            file_path = os.path.join('output', dir_name, file_name) + '.png'
            #p = OrthogonalPolygon.from_positions(vertices)
            #g = nx.compose(g, p)
            #for node in g:

            utils.draw_graph(g, file_path)

            m += 1

    #utils.draw_map(g, [])