#include "LevelMath.h"

namespace level_math
def PointToSegmentSqDistance(self, pt, line):
    if line.GetSqLength() < g_numericalTolerance * g_numericalTolerance:
        return mag2(pt - line.GetPos1())

    d1 = mag2(pt - line.GetPos1())
    d2 = mag2(pt - line.GetPos2())
    pe = line.GetPos2() - line.GetPos1()
    pd = pt - line.GetPos1()
    dp = dot(pe, pd)
    r = dp / mag2(pe)
    float d
    if r >= 1.f:
        d = d2

    elif r <= 0.f:
        d = d1

    else:
        peNew = v2f(pe[1], -pe[0])
        d = std.abs(dot(pd, peNew) / mag(peNew))
        d = d * d

    return d


def PointToLineSqDistance(self, pt, line):
    return PointToLineSqDistance(pt, line.GetPos2(), line.GetPos1())


def PointToLineSqDistance(self, pt, p1, p2):
    pe = p2 - p1
    peNorm = normalize(pe)
    pr = pt - p1
    peNew = v3f(peNorm[0], peNorm[1], 0.f)
    prNew = v3f(pr[0], pr[1], 0.f)
    cp = cross(peNew, prNew)
    d = mag2(cp)
    return d


def RoomPerimeter(self, room1):
    contactArea = 0.f
    for (i = 0; i < room1.GetNumOfEdges(); i++)
        edge1 = room1.GetEdge(i)
        contactArea += edge1.GetLength()


    return contactArea


def RoomContact(self, room1, room2):
    contactArea = 0.f
    for (i = 0; i < room1.GetNumOfEdges(); i++)
        edge1 = room1.GetEdge(i)
        for (j = 0; j < room2.GetNumOfEdges(); j++)
            edge2 = room2.GetEdge(j)
            if edge1.GetDoorFlag() == False or edge2.GetDoorFlag() == False:
                continue

            contactAreaTmp = EdgeContact(edge1, edge2)
            contactArea += contactAreaTmp



    return contactArea


def RoomContact(self, room1, room2, edgeIdx1, edgeIdx2):
    contactAreaMax = 0.f
    for (i = 0; i < room1.GetNumOfEdges(); i++)
        edge1 = room1.GetEdge(i)
        for (j = 0; j < room2.GetNumOfEdges(); j++)
            edge2 = room2.GetEdge(j)
            if edge1.GetDoorFlag() == False or edge2.GetDoorFlag() == False:
                continue

            contactAreaTmp = EdgeContact(edge1, edge2)
            if contactAreaTmp > contactAreaMax:
                contactAreaMax = contactAreaTmp
                edgeIdx1 = i
                edgeIdx2 = j




    return contactAreaMax


def EdgeContact(self, line1, line2):
     numericalTolerance = g_numericalTolerance * 100.f
     numericalToleranceSq = numericalTolerance * numericalTolerance
    pr1 = line1.GetPos2() - line1.GetPos1()
    pr2 = line2.GetPos2() - line2.GetPos1()
    pe1 = v3f(pr1[0], pr1[1], 0.f)
    pe2 = v3f(pr2[0], pr2[1], 0.f)
    cp = cross(pe1, pe2)
    if mag2(cp) > numericalTolerance:
        return 0.f

    posMin1 = min_union(line1.GetPos1(), line1.GetPos2())
    posMax1 = max_union(line1.GetPos1(), line1.GetPos2())
    posMin2 = min_union(line2.GetPos1(), line2.GetPos2())
    posMax2 = max_union(line2.GetPos1(), line2.GetPos2())
    for (j = 0; j < 2; j++)
        if posMax1[j] < posMin2[j] - numericalTolerance or posMin1[j] > posMax2[j] + numericalTolerance:
            return 0.f


    d1 = PointToLineSqDistance(line2.GetPos1(), line1)
    d2 = PointToLineSqDistance(line2.GetPos2(), line1)
    if d1 > numericalToleranceSq or d2 > numericalToleranceSq:
        return 0.f

    # Now the two edges should in the same line anyway...
    len1 = mag(pe1)
    len2 = mag(pe2)
    d11 = mag2(line1.GetPos1() - line2.GetPos1())
    d21 = mag2(line1.GetPos2() - line2.GetPos1())
    d12 = mag2(line1.GetPos1() - line2.GetPos2())
    d22 = mag2(line1.GetPos2() - line2.GetPos2())
    dMax = sqrt(max(max(d11, d21), max(d12, d22)))
    dMax = max(dMax, max(len1, len2))
    contactArea = len1 + len2 - dMax
    contactArea = max(contactArea, 0.f)
    return contactArea


def RoomDistance(self, room1, room2):
    d = 1e10
    for (i = 0; i < room1.GetNumOfVertices(); i++)
        pt = room1.GetVertex(i)
        for (j = 0; j < room2.GetNumOfEdges(); j++)
            edge = room2.GetEdge(j)
            dTmp = PointToSegmentSqDistance(pt, edge)
            d = min(d, dTmp)


    d = sqrt(d)
    return d


def SegmentIntersection(self, pa, pb, pc, pd, pi):
    return SegmentIntersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1], pi[0], pi[1])


# Based on the example under http:#stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def SegmentIntersection(self, Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):
    Rx = Bx - Ax
    Ry = By - Ay
    Sx = Dx - Cx
    Sy = Dy - Cy
    QPx = Cx - Ax
    QPy = Cy - Ay
    rs = Rx * Sy - Ry * Sx
    if rs == 0.f:
        return False

    t = (QPx * Sy - QPy * Sx) / rs
    u = (QPx * Ry - QPy * Rx) / rs
    if t >= 0.f and t <= 1.f and u >= 0.f and u <= 1.f:
        Ix = Ax + t * Rx
        Iy = Ay + t * Ry
        return True

    else:
        return False



def LineIntersection(self, pa, pb, pc, pd, pi):
    return LineIntersection(pa[0], pa[1], pb[0], pb[1], pc[0], pc[1], pd[0], pd[1], pi[0], pi[1])


def LineIntersection(self, Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):
    Rx = Bx - Ax
    Ry = By - Ay
    Sx = Dx - Cx
    Sy = Dy - Cy
    QPx = Cx - Ax
    QPy = Cy - Ay
    rs = Rx * Sy - Ry * Sx
    if rs == 0.f:
        return False

    t = (QPx * Sy - QPy * Sx) / rs
    Ix = Ax + t * Rx
    Iy = Ay + t * Ry
    return True


def ComparePrSmallerFirst(self, pr1, pr2):
    return (pr1.m_dp < pr2.m_dp)


def SortVecPr(self, vecPr):
    if vecPr.size() < 2:
        return

    pd = vecPr[1] - vecPr[0]
    std.vector<PrSort> vecPrSort(vecPr.size())
    for (i = 0; i < int(vecPrSort.size()); i++)
        vecPrSort[i].m_pr = vecPr[i]
        vecPrSort[i].m_dp = dot(pd, vecPr[i] - vecPr[0])

    sort(vecPrSort.begin(), vecPrSort.end(), ComparePrSmallerFirst)
    for (i = 0; i < int(vecPrSort.size()); i++)
        vecPr[i] = vecPrSort[i].m_pr



def randomColorFromIndex(self, idx):
    static std.vector<v3f> clrs
    if clrs.empty():
        clrs.resize(256)
        for (c = 0; c < clrs.size(); c++)
            clrs[c] = v3f(rand(), rand(), rand())
            clrs[c] = clrs[c] / max(clrs[c])


    return (clrs[idx & 255])

} # namespace level_math
