import math
from functools import cmp_to_key

import numpy as np
from dataclasses import dataclass

from reactor.geometry.vector import Vector2, Vector3


NUMERICAL_TOLERANCE = 1e-4
NUMERICAL_TOLERANCE_SQ = NUMERICAL_TOLERANCE * NUMERICAL_TOLERANCE


@dataclass
class PrSort:

    m_pr: Vector2
    m_dp: float


def point_to_segment_sq_distance(pt, line):
    if line.sq_length < NUMERICAL_TOLERANCE_SQ:
        return (pt - line.pos1).mag2()
    d1 = (pt - line.pos1).mag2()
    d2 = (pt - line.pos2).mag2()
    pe = line.pos2 - line.pos1
    pd = pt - line.pos1
    dp = pe.dot(pd)
    r = dp / pe.mag2()
    if r >= 1:
        d = d2
    elif r <= 0:
        d = d1
    else:
        pe_new = Vector2(pe[1], -pe[0])
        d = abs(pd.dot(pe_new) / pe_new.mag())
        d = d * d
    return d


def point_to_line_sq_distance(pt, *args):
    try:
        p1, p2 = args
    except ValueError:
        line = args[0]
        p1, p2 = line.pos2, line.pos1
    pe = p2 - p1
    pe_norm = pe.normalise()
    pr = pt - p1
    pe_new = Vector3(pe_norm[0], pe_norm[1], 0)
    pr_new = Vector3(pr[0], pr[1], 0)
    cp = pe_new.cross(pr_new)
    d = cp.mag2()
    return d


def room_perimeter(room1):
    contact_area = 0
    for i in range(room1.num_edges):
        edge1 = room1.get_edge(i)
        contact_area += edge1.length
    return contact_area


def room_contact(room1, room2):
    contact_area = 0
    for edge1 in room1.get_edges():
        for edge2 in room2.get_edges():
            if not edge1.door_flag or not edge2.door_flag:
                print('    bail bc one is not edge')
                continue
            contact_area += edge_contact(edge1, edge2)
    return contact_area


# def room_contact(room1, room2, edgeIdx1, edgeIdx2):
#     contact_area_max = 0
#     for i in range(room1.num_of_edges):
#         edge1 = room1.get_edge(i)
#         for j in range(room2.num_of_edges):
#             edge2 = room2.get_edge(j)
#             if not edge1.door_flag or not edge2.door_flag:
#                 continue
#
#             contact_area_tmp = edge_contact(edge1, edge2)
#             if contact_area_tmp > contact_area_max:
#                 contact_area_max = contact_area_tmp
#                 edgeIdx1 = i
#                 edgeIdx2 = j
#
#     return contact_area_max


def edge_contact(line1, line2):
    numerical_tolerance = NUMERICAL_TOLERANCE * 100
    numerical_tolerance_sq = numerical_tolerance * numerical_tolerance
    pr1 = line1.pos2 - line1.pos1
    pr2 = line2.pos2 - line2.pos1
    pe1 = Vector3(pr1[0], pr1[1], 0.0)
    pe2 = Vector3(pr2[0], pr2[1], 0.0)
    cp = pe1.cross(pe2)
    if cp.mag2() > numerical_tolerance:
        return 0.0

    pos_min1 = line1.pos1.minimum(line1.pos2)
    pos_max1 = line1.pos1.maximum(line1.pos2)
    pos_min2 = line2.pos1.minimum(line2.pos2)
    pos_max2 = line2.pos1.maximum(line2.pos2)
    for j in range(2):
        if pos_max1[j] < pos_min2[j] - numerical_tolerance or pos_min1[j] > pos_max2[j] + numerical_tolerance:
            return 0.0

    d1 = point_to_line_sq_distance(line2.pos1, line1)
    d2 = point_to_line_sq_distance(line2.pos2, line1)
    if d1 > numerical_tolerance_sq or d2 > numerical_tolerance_sq:
        return 0.0

    # Now the two edges should be in the same line (parallel?).
    len1 = pe1.mag()
    len2 = pe2.mag()
    d11 = (line1.pos1 - line2.pos1).mag2()
    d21 = (line1.pos2 - line2.pos1).mag2()
    d12 = (line1.pos1 - line2.pos2).mag2()
    d22 = (line1.pos2 - line2.pos2).mag2()
    d_max = math.sqrt(max(max(d11, d21), max(d12, d22)))
    d_max = max(d_max, max(len1, len2))
    contact_area = len1 + len2 - d_max
    contact_area = max(contact_area, 0.0)
    return contact_area


def room_distance(room1, room2):
    d = 1e10
    for node in room1.get_nodes():
        for edge in room2.get_edges():
            d_tmp = point_to_segment_sq_distance(node.position, edge)
            d = min(d, d_tmp)
    return math.sqrt(d)


# def segment_intersection(pa, pb, pc, pd, pi):
#     return segment_intersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1], pi[0], pi[1])


# Based on the example under http:#stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def segment_intersection(*args):
    if len(args) == 10:
        Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = args
    else:
        pa, pb, pc, pd = args
        Ax, Ay, Bx, By, Cx, Cy, Dx, Dy = pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1]

    Rx = Bx - Ax
    Ry = By - Ay
    Sx = Dx - Cx
    Sy = Dy - Cy
    QPx = Cx - Ax
    QPy = Cy - Ay
    rs = Rx * Sy - Ry * Sx
    if rs == 0:
        return False

    t = (QPx * Sy - QPy * Sx) / rs
    u = (QPx * Ry - QPy * Rx) / rs
    if t >= 0 and t <= 1 and u >= 0 and u <= 1:
        Ix = Ax + t * Rx
        Iy = Ay + t * Ry
        return True, Vector2(Ix, Iy)
    else:
        return False, (None, None)


def line_intersection(pa, pb, pc, pd, pi):
    return line_intersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1], pi[0], pi[1])


def line_intersection(Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):
    Rx = Bx - Ax
    Ry = By - Ay
    Sx = Dx - Cx
    Sy = Dy - Cy
    QPx = Cx - Ax
    QPy = Cy - Ay
    rs = Rx * Sy - Ry * Sx
    if rs == 0:
        return False

    t = (QPx * Sy - QPy * Sx) / rs
    Ix = Ax + t * Rx
    Iy = Ay + t * Ry
    return True


def compare_pr_smaller_first(pr1, pr2):
    if pr1.m_dp == pr2.m_dp:
        return 0
    else:
        return 1 if pr1.m_dp < pr2.m_dp else -1


def sort_vec_pr(vec_pr):
    if len(vec_pr) < 2:
        return
    pd = vec_pr[1] - vec_pr[0]
    pr_sorts = []
    for i in range(len(vec_pr)):
        pr_sort = PrSort(
            vec_pr[i],
            pd.dot(vec_pr[i] - vec_pr[0])
        )
        pr_sorts.append(pr_sort)
    pr_sorts.sort(key=cmp_to_key(compare_pr_smaller_first))
    for i in range(len(pr_sorts)):
        vec_pr[i] = pr_sorts[i].m_pr


def random_color_from_index(self, idx):
    clrs = Vector3(0, 0, 0)
    if clrs.empty():
        clrs.resize(256)
        for c in range(len(clrs)):
            clrs[c] = Vector3(random.random(), random.random(), random.random())
            clrs[c] = clrs[c] / max(clrs[c])
    return (clrs[idx & 255])
