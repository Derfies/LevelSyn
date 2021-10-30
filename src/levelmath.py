import math

from reactor.geometry.vector import Vector2, Vector3


NUMERICAL_TOLERANCE = 1e-4
NUMERICAL_TOLERANCE_SQ = NUMERICAL_TOLERANCE * NUMERICAL_TOLERANCE


def point_to_segment_sq_distance(pt, line):
    if line.sq_length < NUMERICAL_TOLERANCE * NUMERICAL_TOLERANCE:
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


def point_to_line_sq_distance(pt, line):
    return point_to_line_sq_distance(pt, line.pos2, line.pos1)


def point_to_line_sq_distance(pt, p1, p2):
    pe = p2 - p1
    pe_norm = pe.normalize()
    pr = pt - p1
    pe_new = Vector3(pe_norm[0], pe_norm[1], 0)
    pr_new = Vector3(pr[0], pr[1], 0)
    cp = pe_new.cross(pr_new)
    d = cp.mag2()
    return d


def room_perimeter(room1):
    contact_area = 0
    for i in range(room1.num_of_edges):
        edge1 = room1.get_edge(i)
        contact_area += edge1.length
    return contact_area


def room_contact(room1, room2):
    contact_area = 0
    for i in range(room1.num_of_edges):
        edge1 = room1.get_edge(i)
        for j in range(room2.num_of_edges):
            edge2 = room2.get_edge(j)
            if not edge1.door_flag or not edge2.door_flag:
                continue
            contact_area += edge_contact(edge1, edge2)
    return contact_area


def room_contact(room1, room2, edgeIdx1, edgeIdx2):
    contact_area_max = 0
    for i in range(room1.num_of_edges):
        edge1 = room1.get_edge(i)
        for j in range(room2.num_of_edges):
            edge2 = room2.get_edge(j)
            if not edge1.door_flag or not edge2.door_flag:
                continue

            contact_area_tmp = edge_contact(edge1, edge2)
            if contact_area_tmp > contact_area_max:
                contact_area_max = contact_area_tmp
                edgeIdx1 = i
                edgeIdx2 = j

    return contact_area_max


def edge_contact(line1, line2):
    numericalTolerance = NUMERICAL_TOLERANCE * 100
    numericalToleranceSq = numericalTolerance * numericalTolerance
    pr1 = line1.pos2 - line1.pos1
    pr2 = line2.pos2 - line2.pos1
    pe1 = Vector3(pr1[0], pr1[1], 0.0)
    pe2 = Vector3(pr2[0], pr2[1], 0.0)
    cp = cross(pe1, pe2)
    if mag2(cp) > numericalTolerance:
        return 0.0

    posMin1 = min_union(line1.pos1, line1.pos2)
    posMax1 = max_union(line1.pos1, line1.pos2)
    posMin2 = min_union(line2.pos1, line2.pos2)
    posMax2 = max_union(line2.pos1, line2.pos2)
    for j in range(2):
        if posMax1[j] < posMin2[j] - numericalTolerance or posMin1[j] > posMax2[j] + numericalTolerance:
            return 0.0


    d1 = point_to_line_sq_distance(line2.pos1, line1)
    d2 = point_to_line_sq_distance(line2.pos2, line1)
    if d1 > numericalToleranceSq or d2 > numericalToleranceSq:
        return 0.0

    # Now the two edges should in the same line anyway...
    len1 = mag(pe1)
    len2 = mag(pe2)
    d11 = mag2(line1.pos1 - line2.pos1)
    d21 = mag2(line1.pos2 - line2.pos1)
    d12 = mag2(line1.pos1 - line2.pos2)
    d22 = mag2(line1.pos2 - line2.pos2)
    dMax = math.sqrt(max(max(d11, d21), max(d12, d22)))
    dMax = max(dMax, max(len1, len2))
    contact_area = len1 + len2 - dMax
    contact_area = max(contact_area, 0.0)
    return contact_area


def room_distance(room1, room2):
    d = 1e10
    for i in range(room1.num_vertices):
        pt = room1.get_vertex(i)
        for j in range(room2.num_of_edges):
            edge = room2.get_edge(j)
            d_tmp = point_to_segment_sq_distance(pt, edge)
            d = min(d, d_tmp)
    return math.sqrt(d)


def segment_intersection(pa, pb, pc, pd, pi):
    return segment_intersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1], pi[0], pi[1])


# Based on the example under http:#stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def segment_intersection(Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):
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
        return True
    else:
        return False


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


def ComparePrSmallerFirst(pr1, pr2):
    return (pr1.m_dp < pr2.m_dp)


def sort_vec_pr(vec_pr):
    if vec_pr.size() < 2:
        return

    pd = vec_pr[1] - vec_pr[0]
    std.vector<PrSort> vec_prSort(vec_pr.size())
    for i in range(len(vec_prSort)):
        vec_prSort[i].m_pr = vec_pr[i]
        vec_prSort[i].m_dp = dot(pd, vec_pr[i] - vec_pr[0])

    sort(vec_prSort.begin(), vec_prSort.end(), ComparePrSmallerFirst)
    #for (i = 0; i < int(vec_prSort.size()); i++)
    for i in range(len(vec_prSort)):
        vec_pr[i] = vec_prSort[i].m_pr


def random_color_from_index(self, idx):
    clrs = Vector3(0, 0, 0)
    if clrs.empty():
        clrs.resize(256)
        #for (c = 0; c < clrs.size(); c++)
        for c in range(len(clrs)):
            clrs[c] = Vector3(rand(), rand(), rand())
            clrs[c] = clrs[c] / max(clrs[c])
    return (clrs[idx & 255])
