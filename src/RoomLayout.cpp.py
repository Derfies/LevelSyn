#include "RoomLayout.h"

def GetNumOfVertices(self):
    numOfVertices = 0
    for (i = 0; i < GetNumOfRooms(); i++)
        numOfVertices += m_rooms[i].GetNumOfVertices()

    return numOfVertices


def GetNumOfEdges(self):
    numOfEdges = 0
    for (i = 0; i < GetNumOfRooms(); i++)
        numOfEdges += m_rooms[i].GetNumOfEdges()

    return numOfEdges


def GetNearestEdgePair(self, roomIdx0, roomIdx1):
    v2i pairMin(-1)
    distMin = 1e10
    room0 = GetRoom(roomIdx0)
    room1 = GetRoom(roomIdx1)
    numOfEdges0 = room0.GetNumOfEdges()
    numOfEdges1 = room1.GetNumOfEdges()
    for (i = 0; i < numOfEdges0; i++)
        idx00 = i
        idx01 = (i + 1) % numOfEdges0
        e0 = room0.GetVertex(idx01) - room0.GetVertex(idx00)
        p0 = (room0.GetVertex(idx01) + room0.GetVertex(idx00)) * 0.5f
        for (j = 0; j < numOfEdges1; j++)
            idx10 = j
            idx11 = (j + 1) % numOfEdges1
            e1 = room1.GetVertex(idx11) - room1.GetVertex(idx10)
            p1 = (room1.GetVertex(idx11) + room1.GetVertex(idx10)) * 0.5f
            pr = p1 - p0
            if std.abs(dot(e0, e1)) < 1e-10:
                continue

            distTmp = mag2(pr)
            if distTmp < distMin:
                distMin = distTmp
                pairMin[0] = i
                pairMin[1] = j



    return pairMin


def GetLayoutBoundingBox(self, posMin, posMax):
    v2f pMin(1e10)
    v2f pMax(-1e10)
    for (i = 0; i < GetNumOfRooms(); i++)
        v2f pMinTmp
        v2f pMaxTmp
        GetRoom(i).GetRoomBoundingBox(pMinTmp, pMaxTmp)
        for (j = 0; j < 2; j++)
            pMin[j] = min(pMin[j], pMinTmp[j])
            pMax[j] = max(pMax[j], pMaxTmp[j])


    posMin = pMin
    posMax = pMax


def MoveToSceneCenter(self):
    v2f posMin, posMax
    GetLayoutBoundingBox(posMin, posMax)
    posCen = (posMin + posMax) * 0.5f
    for (i = 0; i < GetNumOfRooms(); i++)
        GetRoom(i).TranslateRoom(-posCen)



def GetRoomPositions(self):
    std.vector<v2f> roomPositions(GetNumOfRooms())
    for (i = 0; i < GetNumOfRooms(); i++)
        roomPositions[i] = GetRoom(i).GetRoomCenter()

    return roomPositions


def ResetRoomEnergies(self):
    for (i = 0; i < GetNumOfRooms(); i++)
        GetRoom(i).ResetEnergy()



def PrintLayout(self):
    std.cout << "A layout with " << GetNumOfRooms() << " rooms...\n"
    for (i = 0; i < GetNumOfRooms(); i++)
        std.cout << i << "th room:\n"
        GetRoom(i).PrintRoom()



#include "PlanarGraph.h"

def SaveLayoutAsSVG(self, fileName, wd ''' = 400 ''', ht ''' = 400 ''', visitedOnly ''' = FALSE ''', CPlanarGraph* graphBest ''' = NULL ''', labelFlag ''' = True '''):
    strokeWd = 4
    v2f posMin, posMax
    GetLayoutBoundingBox(posMin, posMax)
    pMin = min(posMin[0], posMin[1])
    pMax = max(posMax[0], posMax[1])
    '''
	#ifdef DUMP_INTERMEDIATE_OUTPUT
	pMin = -1.f
	pMax = 1.f
	#endif
	'''

    scaling = 1.05f
    pMin *= scaling
    pMax *= scaling
     str = "\t<?xml version=\"1.0\" standalone=\"no\" ?>\n"
                      "<not -- layout visualization -.\n"
                      "<svg>\n"
                      "</svg>\n"
    tinyxml2.XMLDocument doc
    doc.Parse(str)
    root = doc.RootElement()
    std.ostringstream ossViewBox
    ossViewBox << 0 << " " << 0 << " " << wd << " " << ht
    root.SetAttribute("viewBox", ossViewBox.str().c_str())
    root.SetAttribute("xmlns", "http:#www.w3.org/2000/svg")
    # Draw a background...
    bgElement = doc.NewElement("rect")
    bgElement.SetAttribute("x", 0)
    bgElement.SetAttribute("y", 0)
    bgElement.SetAttribute("width", wd)
    bgElement.SetAttribute("height", ht)
    bgElement.SetAttribute("fill", "#00A000")
    bgElement.SetAttribute("stroke", "none")
    #root.InsertEndChild(bgElement)
    # Dump rooms as closed polygons...
    for (i = 0; i < GetNumOfRooms(); i++)
        if graphBest != NULL and graphBest.GetNode(i).GetFlagVisited() == False and visitedOnly:
            continue

        roomElement = doc.NewElement("path")
        std.ostringstream ossPath
        room = GetRoom(i)
        for (j = 0; j < room.GetNumOfVertices(); j++)
            pj = room.GetVertex(j)
            if j == 0:
                ossPath << "M "

            else:
                ossPath << "L "

            ossPath << ConvertPosX(pj[0], pMin, pMax, wd) << " "
            ossPath << ConvertPosY(pj[1], pMin, pMax, ht) << " "

        ossPath << "z"
        roomElement.SetAttribute("d", ossPath.str().c_str())
        if room.GetFlagFixed() == True:
            roomElement.SetAttribute("fill", "#808080")

        elif room.GetBoundaryType() == 2:
            # Corridors...
            roomElement.SetAttribute("fill", "#FAFAFA")

        else:
            roomElement.SetAttribute("fill", "#E0E0E0")

        roomElement.SetAttribute("stroke", "none")
        root.InsertEndChild(roomElement)

    # Dump rooms as sets of line segments...
    for (i = 0; i < GetNumOfRooms(); i++)
        if graphBest != NULL and graphBest.GetNode(i).GetFlagVisited() == False and visitedOnly:
            continue

        roomElement = doc.NewElement("path")
        std.ostringstream ossPath
        room = GetRoom(i)

        if room.HasWalls() == False:
            for (j = 0; j < room.GetNumOfVertices(); j++)
                pj = room.GetVertex(j)
                if j == 0:
                    ossPath << "M "

                else:
                    ossPath << "L "

                ossPath << ConvertPosX(pj[0], pMin, pMax, wd) << " "
                ossPath << ConvertPosY(pj[1], pMin, pMax, ht) << " "

            ossPath << "z"

        else:
            for (j = 0; j < room.GetNumOfWalls(); j++)
                wall = room.GetWall(j)
                p1 = wall.GetPos1()
                p2 = wall.GetPos2()
                ossPath << "M "
                ossPath << ConvertPosX(p1[0], pMin, pMax, wd) << " "
                ossPath << ConvertPosY(p1[1], pMin, pMax, ht) << " "
                ossPath << "L "
                ossPath << ConvertPosX(p2[0], pMin, pMax, wd) << " "
                ossPath << ConvertPosY(p2[1], pMin, pMax, ht) << " "


        roomElement.SetAttribute("d", ossPath.str().c_str())
        roomElement.SetAttribute("fill", "none")
        roomElement.SetAttribute("stroke", "black")
        roomElement.SetAttribute("stroke-width", strokeWd)
        root.InsertEndChild(roomElement)

    # Dump vertices...
    for (i = 0; i < GetNumOfRooms(); i++)
        if graphBest != NULL and graphBest.GetNode(i).GetFlagVisited() == False and visitedOnly:
            continue

        room = GetRoom(i)
        for (j = 0; j < room.GetNumOfVertices(); j++)
            vertexElement = doc.NewElement("circle")
            pj = room.GetVertex(j)
            vertexElement.SetAttribute("cx", ConvertPosX(pj[0], pMin, pMax, wd))
            vertexElement.SetAttribute("cy", ConvertPosY(pj[1], pMin, pMax, ht))
            vertexElement.SetAttribute("r", strokeWd / 2)
            vertexElement.SetAttribute("fill", "black")
            vertexElement.SetAttribute("stroke", "none")
            root.InsertEndChild(vertexElement)


    # Dump corridor walls...
    for (i = 0; i < GetNumOfCorridorWalls(); i++)
        wall = GetCorridorWall(i)
        p1 = wall.GetPos1()
        p2 = wall.GetPos2()
        wallElement = doc.NewElement("path")
        std.ostringstream ossWall
        ossWall << "M "
        ossWall << CRoomLayout.ConvertPosX(p1[0], pMin, pMax, wd) << " "
        ossWall << CRoomLayout.ConvertPosY(p1[1], pMin, pMax, ht) << " "
        ossWall << "L "
        ossWall << CRoomLayout.ConvertPosX(p2[0], pMin, pMax, wd) << " "
        ossWall << CRoomLayout.ConvertPosY(p2[1], pMin, pMax, ht) << " "
        wallElement.SetAttribute("d", ossWall.str().c_str())
        wallElement.SetAttribute("fill", "none")
        wallElement.SetAttribute("stroke", "black")
        wallElement.SetAttribute("stroke-width", strokeWd)
        root.InsertEndChild(wallElement)
        vertexElement1 = doc.NewElement("circle")
        vertexElement1.SetAttribute("cx", ConvertPosX(p1[0], pMin, pMax, wd))
        vertexElement1.SetAttribute("cy", ConvertPosY(p1[1], pMin, pMax, ht))
        vertexElement1.SetAttribute("r", strokeWd / 2)
        vertexElement1.SetAttribute("fill", "black")
        vertexElement1.SetAttribute("stroke", "none")
        root.InsertEndChild(vertexElement1)
        vertexElement2 = doc.NewElement("circle")
        vertexElement2.SetAttribute("cx", ConvertPosX(p2[0], pMin, pMax, wd))
        vertexElement2.SetAttribute("cy", ConvertPosY(p2[1], pMin, pMax, ht))
        vertexElement2.SetAttribute("r", strokeWd / 2)
        vertexElement2.SetAttribute("fill", "black")
        vertexElement2.SetAttribute("stroke", "none")
        root.InsertEndChild(vertexElement2)

    # Dump labels...
    for (i = 0; i < GetNumOfRooms(); i++)
        if graphBest != NULL and graphBest.GetNode(i).GetFlagVisited() == False and visitedOnly:
            continue


        shiftX = (i >= 10) ? 8 : 3
        shiftY = 5
        pi = GetRoom(i).GetRoomCenter()
        pi = pi + GetRoom(i).GetCenterShift()
        labelElement = doc.NewElement("text")
        labelElement.SetAttribute("x", ConvertPosX(pi[0], pMin, pMax, wd) - shiftX)
        labelElement.SetAttribute("y", ConvertPosY(pi[1], pMin, pMax, ht) + shiftY)
        labelElement.SetAttribute("font-family", "Verdana")
        labelElement.SetAttribute("font-size", 15)
        labelElement.SetAttribute("fill", "blue")
        std.ostringstream ossLabel
        ossLabel << i
        labelText = doc.NewText(ossLabel.str().c_str())
        labelElement.InsertEndChild(labelText)
        if labelFlag == True:
            root.InsertEndChild(labelElement)



    saveFlag = doc.SaveFile(fileName)
    return saveFlag


def ConvertPos(self, p, pMin, pMax, sz):
    pd = (p - pMin) / (pMax - pMin) * sz
    return int(pd)


def ConvertPosX(self, p, pMin, pMax, sz):
    return ConvertPos(p, pMin, pMax, sz)


def ConvertPosY(self, p, pMin, pMax, sz):
    return sz - 1 - ConvertPos(p, pMin, pMax, sz)

