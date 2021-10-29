#include "Room.h"

#include "LevelConfig.h"

CRoom.CRoom()
    m_templateType = -1
    m_flagFixed = False
    m_boundaryType = 0
    ResetEnergy()


def GetEdge(self, idx):
    idx1 = idx
    idx2 = (idx + 1) % GetNumOfVertices()
    CRoomEdge edge
    edge.SetPos1(GetVertex(idx1))
    edge.SetPos2(GetVertex(idx2))
    edge.SetIdx1(idx1)
    edge.SetIdx2(idx2)
    edge.SetDoorFlag(GetDoorFlag(idx))
    return edge


def GetRoomCenter(self):
    v2f center(0.f)
    if GetNumOfVertices() == 0:
        return center

    v2f posMin(1e10)
    v2f posMax(-1e10)
    for (i = 0; i < GetNumOfVertices(); i++)
        pi = m_vertices[i]
        for (j = 0; j < 2; j++)
            posMin[j] = min(posMin[j], pi[j])
            posMax[j] = max(posMax[j], pi[j])


    center = (posMin + posMax) * 0.5f
    return center


def GetShiftedRoomCenter(self):
    center = GetRoomCenter() + m_centerShift
    return center


def TranslateRoom(self, trans):
    for (i = 0; i < GetNumOfVertices(); i++)
        m_vertices[i] = m_vertices[i] + trans



def RotateRoom(self, rad):
    cv = cos(rad)
    sv = sin(rad)
    for (i = 0; i < GetNumOfVertices(); i++)
        p0 = m_vertices[i][0]
        p1 = m_vertices[i][1]
        m_vertices[i][0] = p0 * cv + p1 * sv
        m_vertices[i][1] = -p0 * sv + p1 * cv

    p0 = m_centerShift[0]
    p1 = m_centerShift[1]
    m_centerShift[0] = p0 * cv + p1 * sv
    m_centerShift[1] = -p0 * sv + p1 * cv


def ScaleRoom(self, scaling):
    center = GetRoomCenter()
    for (i = 0; i < GetNumOfVertices(); i++)
        pi = m_vertices[i] - center
        m_vertices[i] = center + pi * scaling

    m_centerShift = m_centerShift * scaling


def ScaleRoom(self, scaling):
    center = GetRoomCenter()
    for (i = 0; i < GetNumOfVertices(); i++)
        pi = m_vertices[i] - center
        pi[0] *= scaling[0]
        pi[1] *= scaling[1]
        m_vertices[i] = center + pi

    m_centerShift[0] *= scaling[0]
    m_centerShift[1] *= scaling[1]


def GetRoomBoundingBox(self, posMin, posMax):
    v2f pMin(1e10)
    v2f pMax(-1e10)
    for (i = 0; i < GetNumOfVertices(); i++)
        pi = m_vertices[i]
        for (j = 0; j < 2; j++)
            pMin[j] = min(pMin[j], pi[j])
            pMax[j] = max(pMax[j], pi[j])


    posMin = pMin
    posMax = pMax


def GetRoomBoundingBox(self, boundingBox):
    v2f posMin, posMax
    GetRoomBoundingBox(posMin, posMax)
    boundingBox.m_posMin = posMin
    boundingBox.m_posMax = posMax


def PrintRoom(self):
    std.cout << "A room with " << GetNumOfVertices() << " vertices...\n"
    for (i = 0; i < GetNumOfVertices(); i++)
        std.cout << i << "th vertex: " << GetVertex(i) << std.endl



def InitWalls(self):
    m_walls.clear()
    for (i = 0; i < GetNumOfVertices(); i++)
        idx1 = i
        idx2 = (i + 1) % GetNumOfVertices()
        pos1 = GetVertex(idx1)
        pos2 = GetVertex(idx2)
        RoomWall wall(pos1, pos2)
        m_walls.push_back(wall)



def EraseWall(self, idx):
    if idx >= GetNumOfWalls():
        return False

    m_walls.erase(m_walls.begin() + idx)
    return True


def ResetDoorFlags(self):
    if m_doorFlags.empty() == False:
        return

    m_doorFlags.resize(GetNumOfEdges(), False)


def SetDoorFlag(self, edgeIdx, doorFlag):
    if edgeIdx < 0 or edgeIdx >= int(m_doorFlags.size()):
        return

    m_doorFlags[edgeIdx] = doorFlag


def GetDoorFlag(self, edgeIdx):
    if edgeIdx < 0 or edgeIdx >= int(m_doorFlags.size()):
        return True

    return m_doorFlags[edgeIdx]


def GetDoorFlags(self):
    return m_doorFlags


def HasRestrictedDoorPosition(self):
    for (i = 0; i < int(m_doorFlags.size()); i++)
        if m_doorFlags[i] == False:
            return True


    return False

