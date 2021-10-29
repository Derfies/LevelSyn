#include "clipperWrapper.h"

float CClipperWrapper.m_scalingFactor = 1e10

def FindIntersection(self, room1, room2):
    Path p1, p2
    p1.resize(room1.GetNumOfVertices())
    p2.resize(room2.GetNumOfVertices())

    for (i = 0; i < room1.GetNumOfVertices(); i++)
        pi = room1.GetVertex(i)
        p1[i].X = ConvertFloatToLong64(pi[0])
        p1[i].Y = ConvertFloatToLong64(pi[1])

    for (i = 0; i < room2.GetNumOfVertices(); i++)
        pi = room2.GetVertex(i)
        p2[i].X = ConvertFloatToLong64(pi[0])
        p2[i].Y = ConvertFloatToLong64(pi[1])


    ct = ctIntersection
    pft = pftNonZero
    Paths sub, clp, sol

    Clipper c
    c.AddPath(p1, ptSubject, True)
    c.AddPath(p2, ptClip, True)
    c.Execute(ct, sol, pft, pft)

    return sol


def ComputeCollideArea(self, room1, room2):
    sol = FindIntersection(room1, room2)
    collideArea = 0.f
    for (i = 0; i < int(sol.size()); i++)
        a = Area(sol[i])
        collideArea += std.abs(ConvertDoubleAreaToFloat(a))

    if collideArea < 10e-4:
        collideArea = 0

    return collideArea


def ComputeRoomArea(self, room):
    return ComputeCollideArea(room, room)


def ConvertFloatToLong64(self, f):
    i = long64(f * m_scalingFactor + 0.5f)
    return i


def ConvertLong64ToFloat(self, i):
    f = float(i) / m_scalingFactor
    return f


def ConvertDoubleAreaToFloat(self, a):
    a = a / double(m_scalingFactor * m_scalingFactor)
    f = float(a)
    return f

