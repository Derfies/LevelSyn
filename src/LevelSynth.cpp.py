#include "LevelSynth.h"

#include <math.h>

def Random2(self, max):
    if max < 1 or max >= RAND_MAX:
        return 0
    else:
        return (int)rand() / (RAND_MAX / max + 1)


def MoveRoomsToSceneCenter(self, ptrGraph):
    v2f pMin(1e10)
    v2f pMax(-1e10)
    for (j = 0; j < ptrGraph.GetNumOfNodes(); j++)
        if ptrGraph.GetNode(j).GetFlagVisited() == False:
            continue

        pj = m_stateRoomPositions[j]
        for (k = 0; k < 2; k++)
            pMin[k] = min(pMin[k], pj[k])
            pMax[k] = max(pMax[k], pj[k])


    posCen = (pMin + pMax) * 0.5f
    for (j = 0; j < ptrGraph.GetNumOfNodes(); j++)
        m_stateRoomPositions[j] = m_stateRoomPositions[j] - posCen



def Move1DchainToSceneCenter(self, indices):
    v2f pMin(1e10)
    v2f pMax(-1e10)
    for (j = 0; j < int(indices.size()); j++)
        idx = indices[j]
        pj = m_stateRoomPositions[idx]
        for (k = 0; k < 2; k++)
            pMin[k] = min(pMin[k], pj[k])
            pMax[k] = max(pMax[k], pj[k])


    posCen = (pMin + pMax) * 0.5f
    for (j = 0; j < int(indices.size()); j++)
        idx = indices[j]
        m_stateRoomPositions[idx] = m_stateRoomPositions[idx] - posCen



def GetStateDifference(self, otherState, ptrGraph):
    stateDiff = 0.f
    for (j = 0; j < ptrGraph.GetNumOfNodes(); j++)
        # ptrGraph.GetNode(pickedRoomIndex).SetType(typeNew)

        if ptrGraph.GetNode(j).GetFlagVisited() == False:
            continue

        '''
		if ptrGraph.GetNode(j).GetType() != otherState.m_stateGraph.GetNode(j).GetType():
		stateDiff += 2 * CLevelConfig.m_stateDiffThresh

		'''
        p1 = m_stateRoomPositions[j]
        p2 = otherState.m_stateRoomPositions[j]
        stateDiff += mag2(p1 - p2)

    return stateDiff


def InsertToNewStates(self, newStates, ptrGraph):
    stateDiffThresh = CLevelConfig.m_stateDiffThresh
    if stateDiffThresh <= 0.f:
        newStates.push_back(*self)
        return True

    for (i = 0; i < int(newStates.size()); i++)
        if m_stateEnergy < newStates[i].m_stateEnergy:
            continue

        stateDiff = GetStateDifference(newStates[i], ptrGraph)
        if stateDiff <= stateDiffThresh:
            return False


    newStates.push_back(*self)
    return True


CLevelSynth.CLevelSynth()
    m_ptrGraph = NULL
    m_ptrTemplates = NULL
    m_solutionCount = 0
    m_bestSolCount = 0
    m_chainCount = 0
    m_backTrackCount = 0
    m_backTrackLevel = 0


CLevelSynth.CLevelSynth(CPlanarGraph* ptrGraph, ptrTemplates)
    SetGraphAndTemplates(ptrGraph, ptrTemplates)


def SetGraphAndTemplates(self, ptrGraph, ptrTemplates):
    m_solutionCount = 0
    m_bestSolCount = 0
    ptrGraph.MoveGraphToSceneCenter()
    ptrGraph.ScaleGraphNodePositions(CLevelConfig.m_graphScaling)
    SetGraph(ptrGraph)
    m_ptrTemplates = ptrTemplates
    m_ptrGraph.SetNumOfTypes(m_ptrTemplates.GetNumOfTemplates())
    m_ptrGraph.RandomInitTypes()

    #m_ptrGraph.RandomInitPositions()
    InitScene()
    SynthesizeScene()


def SetGraph(self, ptrGraph):
    m_ptrGraph = ptrGraph
    m_roomPositions.resize(m_ptrGraph.GetNumOfNodes())
    for (i = 0; i < m_ptrGraph.GetNumOfNodes(); i++)
        pi = m_ptrGraph.GetNodePos(i)
        m_roomPositions[i] = pi



def MovePickedGraphNode(self, dx, dy):
    m_ptrGraph.MovePickedNode(dx, dy)
    InitScene()
    flag = False
    #flag = AdjustPickedRoom(dx, dy)
    return flag


def AdjustPickedRoom(self, dx, dy):
    pickedNodeIndex = m_ptrGraph.GetPickedNodeIndex()
    flag = False
    if pickedNodeIndex < 0:
        return flag

    pickedRoom = m_layout.GetRoom(pickedNodeIndex)
    for (i = 0; i < pickedRoom.GetNumOfEdges(); i++)
        edge = pickedRoom.GetEdge(i)
        pr2 = edge.GetPos2() - edge.GetPos1()
        pr = v3f(pr2[0], pr2[1], 0.f)
        norm = v2f(pr[1], -pr[0])
        norm = normalize(norm)
        distMin = 1e10
        CRoomEdge edgeNearest
        for (j = 0; j < m_layout.GetNumOfRooms(); j++)
            if j == pickedNodeIndex:
                continue

            otherRoom = m_layout.GetRoom(j)
            for (k = 0; k < otherRoom.GetNumOfEdges(); k++)
                otherEdge = otherRoom.GetEdge(k)
                otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
                otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
                cp = cross(pr, otherPr)
                if mag2(cp) > 0.0001f:
                    continue

                prTmp = otherEdge.GetPos1() - edge.GetPos1()
                distTmp = std.abs(dot(norm, prTmp))
                if distTmp < distMin:
                    distMin = distTmp
                    edgeNearest = otherEdge



        if distMin < 0.05f:
            pr = edgeNearest.GetPos1() - edge.GetPos1()
            d = dot(norm, pr)
            dp = d * norm
            m_ptrGraph.MovePickedNode(dp)
            InitScene()
            dx += dp[0]
            dy += dp[1]
            flag = True


    return flag


def InitScene(self):
    m_layout.ClearLayout()
    numOfRooms = m_ptrGraph.GetNumOfNodes()
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    for (i = 0; i < numOfRooms; i++)
        #idx = int(rand() / float(RAND_MAX) * numOfTemplates)
        idx = m_ptrGraph.GetNode(i).GetType()
        idx = idx % numOfTemplates
        room = m_ptrTemplates.GetRoom(idx)
        #room.ScaleRoom(0.5f)
        pi = m_ptrGraph.GetNodePos(i)
        c = room.GetRoomCenter()
        trans = pi - c
        room.TranslateRoom(trans)
        color = randomColorFromIndex(i)
        if mag2(color) > 2.5f:
            color = color * 0.5f

        room.SetColor(color)
        m_layout.AddRoom(room)



def GetLayout(self, ptrGraph, roomPositions):
    CRoomLayout layout
    numOfRooms = ptrGraph.GetNumOfNodes()
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    for (i = 0; i < numOfRooms; i++)
        idx = ptrGraph.GetNode(i).GetType()
        idx = idx % numOfTemplates
        room = m_ptrTemplates.GetRoom(idx)
        pi = roomPositions[i]
        c = room.GetRoomCenter()
        trans = pi - c
        room.TranslateRoom(trans)
        color = randomColorFromIndex(i)
        if mag2(color) > 2.5f:
            color = color * 0.5f

        room.SetColor(color)
        room.SetFlagFixed(ptrGraph.GetNode(i).GetFlagFixed())
        layout.AddRoom(room)

    return layout


def SynthesizeScene(self):
    SynthesizeSceneViaMainLoop()
    UpdateGraphFromLayout()


def UpdateGraphFromLayout(self):
    numOfRooms = m_ptrGraph.GetNumOfNodes()
    for (i = 0; i < numOfRooms; i++)
        roomCenter = m_layout.GetRoom(i).GetRoomCenter()
        m_ptrGraph.GetNode(i).SetPos(roomCenter)



def PostProcessing(self, layout, ptrGraph):
    for (i = 0; i < layout.GetNumOfRooms(); i++)
        std.vector<int> neighbors
        for (j = 0; j < ptrGraph.GetNumOfEdges(); j++)
            edge = ptrGraph.GetEdge(j)
            if edge.GetIdx0() == i:
                neighbors.push_back(edge.GetIdx1())

            elif edge.GetIdx1() == i:
                neighbors.push_back(edge.GetIdx0())


        if neighbors.empty() == True:
            continue

        CConfigSpace configSpace0(layout.GetRoom(neighbors[0]), layout.GetRoom(i))
        for (j = 1; j < int(neighbors.size()); j++)
            CConfigSpace configSpace1(layout.GetRoom(neighbors[j]), layout.GetRoom(i))
            configSpace0 = CConfigSpace.FindIntersection(configSpace0, configSpace1)

        configSpace0.SelfMerge()
        std.cout << "Size of configuration space for the " << i << "th room: " << configSpace0.GetConfigSpaceSize() << std.endl
        configSpace0.PrintConfigSpace()

    return True


def OpenDoors(self, layout, ptrGraph, flagPartial ''' = False '''):
    for (i = 0; i < layout.GetNumOfRooms(); i++)
        layout.GetRoom(i).InitWalls()

    for (i = 0; i < ptrGraph.GetNumOfEdges(); i++)
        ge = ptrGraph.GetEdge(i)
        roomIdx1 = ge.GetIdx0()
        roomIdx2 = ge.GetIdx1()
        room1 = layout.GetRoom(roomIdx1)
        room2 = layout.GetRoom(roomIdx2)
        if room1.GetFlagFixed() == True or room2.GetFlagFixed() == True:
            continue

        int edgeIdx1, edgeIdx2
        contact = RoomContact(room1, room2, edgeIdx1, edgeIdx2)
        if (contact < CLevelConfig.m_roomContactThresh) # just to double check
            if flagPartial == False:
                std.cout << "Failed to open the door on the wall between Room " << roomIdx1 << " and Room " << roomIdx2 << " (case 1)not \n"
                return False

            else:
                continue


        std.vector<v2f> vecPos(4)
        edge1 = room1.GetEdge(edgeIdx1)
        edge2 = room2.GetEdge(edgeIdx2)
        vecPos[0] = edge1.GetPos1()
        vecPos[1] = edge1.GetPos2()
        vecPos[2] = edge2.GetPos1()
        vecPos[3] = edge2.GetPos2()
        SortVecPr(vecPos)
        RoomDoor door(vecPos[1], vecPos[2])
        flag1 = OpenDoor(room1, door)
        flag2 = OpenDoor(room2, door)
        if flag1 == False or flag2 == False:
            std.cout << "Failed to open the door on the wall between Room " << roomIdx1 << " and Room " << roomIdx2 << " (case 2)not \n"
            return False


    return True


def OpenDoor(self, room, door, width ''' = -1.f '''):
     numericalTolerance = g_numericalTolerance * 100.f
     numericalToleranceSq = numericalTolerance * numericalTolerance
     doorWidth = (width > 0.f) ? width : (CLevelConfig.m_roomContactThresh * 0.8f)
    openFlag = False
    for (i = 0; i < room.GetNumOfWalls(); i++)
        wall = room.GetWall(i)
        d1 = PointToSegmentSqDistance(door.GetPos1(), wall)
        d2 = PointToSegmentSqDistance(door.GetPos2(), wall)
        if d1 > numericalToleranceSq or d2 > numericalToleranceSq:
            continue

        room.EraseWall(i)
        d11 = mag2(door.GetPos1() - wall.GetPos1())
        d12 = mag2(door.GetPos2() - wall.GetPos1())
        p1 = (d11 < d12) ? door.GetPos1() : door.GetPos2()
        p2 = (d11 < d12) ? door.GetPos2() : door.GetPos1()
        pMean = (p1 + p2) * 0.5f
        pd = p2 - p1
        pd = normalize(pd) * doorWidth * 0.5f
        p1 = pMean - pd
        p2 = pMean + pd
        RoomWall wall1(wall.GetPos1(), p1)
        RoomWall wall2(p2, wall.GetPos2())
        room.InsertWall(wall1)
        room.InsertWall(wall2)
        openFlag = True
        break

    return openFlag


def OpenDoors(self, layout, layoutShrinked, ptrGraph, thrinkDist):
    for (i = 0; i < layout.GetNumOfRooms(); i++)
        layout.GetRoom(i).InitWalls()
        layoutShrinked.GetRoom(i).InitWalls()

    doorWidth = CLevelConfig.m_roomContactThresh * 0.8f
    for (i = 0; i < ptrGraph.GetNumOfEdges(); i++)
        ge = ptrGraph.GetEdge(i)
        roomIdx1 = ge.GetIdx0()
        roomIdx2 = ge.GetIdx1()
        room1 = layout.GetRoom(roomIdx1)
        room2 = layout.GetRoom(roomIdx2)
        if room1.GetFlagFixed() == True or room2.GetFlagFixed() == True:
            continue

        int edgeIdx1, edgeIdx2
        contact = RoomContact(room1, room2, edgeIdx1, edgeIdx2)
        if (contact < doorWidth) # just to double check
            std.cout << "Failed to open the door on the wall between Room " << roomIdx1 << " and Room " << roomIdx2 << " (case 1)not \n"
            return False

        std.vector<v2f> vecPos(4)
        edge1 = room1.GetEdge(edgeIdx1)
        edge2 = room2.GetEdge(edgeIdx2)
        vecPos[0] = edge1.GetPos1()
        vecPos[1] = edge1.GetPos2()
        vecPos[2] = edge2.GetPos1()
        vecPos[3] = edge2.GetPos2()
        SortVecPr(vecPos)
        p1 = vecPos[1]
        p2 = vecPos[2]
        pr = p2 - p1
        pr = normalize(pr)
        prOrtho = v2f(pr[1], -pr[0])
        pr = pr * doorWidth * 0.5f
        pAve = (p1 + p2) * 0.5f
        p1 = pAve - pr
        p2 = pAve + pr
        p3 = p1 + prOrtho * thrinkDist
        p4 = p1 - prOrtho * thrinkDist
        p5 = p2 + prOrtho * thrinkDist
        p6 = p2 - prOrtho * thrinkDist
        CorridorWall wall1(p3, p4)
        CorridorWall wall2(p5, p6)
        layoutShrinked.InsertCorridorWall(wall1)
        layoutShrinked.InsertCorridorWall(wall2)
        RoomDoor door1(p3, p5)
        RoomDoor door2(p4, p6)
        flag1 = OpenDoor(layoutShrinked.GetRoom(roomIdx1), door1, doorWidth) or OpenDoor(layoutShrinked.GetRoom(roomIdx1), door2, doorWidth)
        flag2 = OpenDoor(layoutShrinked.GetRoom(roomIdx2), door1, doorWidth) or OpenDoor(layoutShrinked.GetRoom(roomIdx2), door2, doorWidth)
        if flag1 == False or flag2 == False:
            std.cout << "Failed to open the door on the wall between Room " << roomIdx1 << " and Room " << roomIdx2 << " (case 2)not \n"
            return False


    return True


def ShrinkRooms(self, layout, dist):
    if dist <= 0.f:
        return

    for (i = 0; i < layout.GetNumOfRooms(); i++)
        ShrinkRoom(layout.GetRoom(i), dist)



def ShrinkRoom(self, room, dist):
    if dist <= 0.f:
        return

     numOfEdges = room.GetNumOfEdges()
    std.vector<CRoomEdge> vecRoomEdge(numOfEdges)
    for (i = 0; i < numOfEdges; i++)
        vecRoomEdge[i] = room.GetEdge(i)

    std.vector<CRoomEdge> vecRoomEdge1 = vecRoomEdge
    std.vector<CRoomEdge> vecRoomEdge2 = vecRoomEdge
    for (i = 0; i < numOfEdges; i++)
        pr = vecRoomEdge[i].GetPos2() - vecRoomEdge[i].GetPos1()
        prNew = v2f(pr[1], -pr[0])
        prNew = normalize(prNew) * dist
        vecRoomEdge1[i].SetPos1(vecRoomEdge1[i].GetPos1() + prNew)
        vecRoomEdge1[i].SetPos2(vecRoomEdge1[i].GetPos2() + prNew)
        vecRoomEdge2[i].SetPos1(vecRoomEdge2[i].GetPos1() - prNew)
        vecRoomEdge2[i].SetPos2(vecRoomEdge2[i].GetPos2() - prNew)

    std.vector<v2f> vertices1 = room.GetVertices()
    std.vector<v2f> vertices2 = room.GetVertices()
    for (i = 0; i < numOfEdges; i++)
        idx1 = i
        idx2 = (i + 1) % numOfEdges
        v2f pi1
        p111 = vecRoomEdge1[idx1].GetPos1()
        p112 = vecRoomEdge1[idx1].GetPos2()
        p121 = vecRoomEdge1[idx2].GetPos1()
        p122 = vecRoomEdge1[idx2].GetPos2()
        LineIntersection(p111, p112, p121, p122, pi1)
        vertices1[i] = pi1
        v2f pi2
        p211 = vecRoomEdge2[idx1].GetPos1()
        p212 = vecRoomEdge2[idx1].GetPos2()
        p221 = vecRoomEdge2[idx2].GetPos1()
        p222 = vecRoomEdge2[idx2].GetPos2()
        LineIntersection(p211, p212, p221, p222, pi2)
        vertices2[i] = pi2

    CClipperWrapper wrapper
    CRoom room1
    room1.SetVertices(vertices1)
    area1 = wrapper.ComputeRoomArea(room1)
    CRoom room2
    room2.SetVertices(vertices2)
    area2 = wrapper.ComputeRoomArea(room2)
    std.vector<v2f> verticesNew = (area1 < area2) ? vertices1 : vertices2
    room.SetVertices(verticesNew)


def SaveGraphAsSVG(self, fileName, ptrGraph, wd ''' = 400 ''', ht ''' = 400 ''', labelRad ''' = 0.25f '''):
    graphNew = *ptrGraph
    graphNew.MoveGraphToSceneCenter()
    strokeWd = 5
    circleRad = 7
    v2f posMin, posMax
    graphNew.GetGraphBoundingBox(posMin, posMax)
    pMin = min(posMin[0], posMin[1])
    pMax = max(posMax[0], posMax[1])
    scaling = 1.05f
    pMin *= scaling
    pMax *= scaling
     str = "\t<?xml version=\"1.0\" standalone=\"no\" ?>\n"
                      "<not -- graph visualization -.\n"
                      "<svg>\n"
                      "</svg>\n"
    tinyxml2.XMLDocument doc
    doc.Parse(str)
    root = doc.RootElement()
    std.ostringstream ossViewBox
    ossViewBox << 0 << " " << 0 << " " << wd << " " << ht
    root.SetAttribute("viewBox", ossViewBox.str().c_str())
    root.SetAttribute("xmlns", "http:#www.w3.org/2000/svg")
    # Dump edges...
    for (i = 0; i < graphNew.GetNumOfEdges(); i++)
        edgeElement = doc.NewElement("path")
        std.ostringstream ossPath
        edge = graphNew.GetEdge(i)
        p1 = graphNew.GetNodePos(edge.GetIdx0())
        p2 = graphNew.GetNodePos(edge.GetIdx1())
        ossPath << "M "
        ossPath << CRoomLayout.ConvertPosX(p1[0], pMin, pMax, wd) << " "
        ossPath << CRoomLayout.ConvertPosY(p1[1], pMin, pMax, ht) << " "
        ossPath << "L "
        ossPath << CRoomLayout.ConvertPosX(p2[0], pMin, pMax, wd) << " "
        ossPath << CRoomLayout.ConvertPosY(p2[1], pMin, pMax, ht) << " "
        edgeElement.SetAttribute("d", ossPath.str().c_str())
        edgeElement.SetAttribute("fill", "none")
        edgeElement.SetAttribute("stroke", "black")
        edgeElement.SetAttribute("stroke-width", strokeWd)
        root.InsertEndChild(edgeElement)

    # Dump nodes...
    for (i = 0; i < graphNew.GetNumOfNodes(); i++)
        nodeElement = doc.NewElement("circle")
        pi = graphNew.GetNodePos(i)
        nodeElement.SetAttribute("cx", CRoomLayout.ConvertPosX(pi[0], pMin, pMax, wd))
        nodeElement.SetAttribute("cy", CRoomLayout.ConvertPosY(pi[1], pMin, pMax, ht))
        nodeElement.SetAttribute("r", circleRad)
        nodeElement.SetAttribute("fill", "red")
        nodeElement.SetAttribute("stroke", "none")
        root.InsertEndChild(nodeElement)

    # Dump labels...
    for (i = 0; i < graphNew.GetNumOfNodes(); i++)
        shiftX = (i >= 10) ? 8 : 3
        shiftY = 5
        pi = ComputeLabelPosition(i, &graphNew, labelRad)
        labelElement = doc.NewElement("text")
        labelElement.SetAttribute("x", CRoomLayout.ConvertPosX(pi[0], pMin, pMax, wd) - shiftX)
        labelElement.SetAttribute("y", CRoomLayout.ConvertPosY(pi[1], pMin, pMax, ht) + shiftY)
        labelElement.SetAttribute("font-family", "Verdana")
        labelElement.SetAttribute("font-size", 13)
        labelElement.SetAttribute("fill", "blue")
        std.ostringstream ossLabel
        ossLabel << i
        labelText = doc.NewText(ossLabel.str().c_str())
        labelElement.InsertEndChild(labelText)
        root.InsertEndChild(labelElement)


    saveFlag = doc.SaveFile(fileName)
    return saveFlag


def CompareStateEnergySmallerFirst(self, state1, state2):
    return (state1.m_stateEnergy < state2.m_stateEnergy)


def SynthesizeSceneViaMainLoop(self):
    CurrentState state0
    state0.m_stateGraph = *m_ptrGraph
    state0.m_stateRoomPositions = m_roomPositions
    state0.m_stateEnergy = 1e10
    std.stack<CurrentState> stateStack
    stateStack.push(state0)
    targetNumOfSolutions = CLevelConfig.m_targetNumOfSolutions
    energyMin = 1e10
    layoutBest = m_layout
    numPartials = 0
    m_backTrackCount = 0
    m_backTrackLevel = 0
    while (m_solutionCount < targetNumOfSolutions and stateStack.empty() == False)
        oldState = stateStack.top()
        stateStack.pop()
        SetCurrentState(oldState)
        m_flagVisitedNoNode = m_ptrGraph.VisitedNoNode()
        flagCyclic = False
        std.vector<int> tmpIndices = m_ptrGraph.ExtractDeepestFaceOrChain(flagCyclic, CLevelConfig.m_flagSmallFaceFirst)
        std.vector<int> indices
#if 0 # Before 09/03/2013
		if  CLevelConfig.m_synMethod != 0 :
			# Select all the graph nodes...
			indices.resize(m_ptrGraph.GetNumOfNodes())
			for ( int i=0; i<int(indices.size()); i++ )
				indices[i] = i


#else:
        indices = oldState.myIndices
        if CLevelConfig.m_synMethod != 0:
            if m_ptrGraph.HasFixedNode() == False or m_ptrGraph.VisitedNoNode() == False:
                indices = m_ptrGraph.GetUnfixedNodes()


        for (i = 0; i < tmpIndices.size(); i++)
            indices.push_back(tmpIndices[i])

#endif
        SetVisitedNeighbors(indices)
        for (i = 0; i < m_ptrGraph.GetNumOfNodes(); i++)
            m_ptrGraph.GetNode(i).SetFlagVisited(False)


        for (i = 0; i < int(indices.size()); i++)
            index = indices[i]
            m_ptrGraph.GetNode(indices[i]).SetFlagVisited(True)

        oldState.m_stateGraph = *m_ptrGraph
        std.vector<CurrentState> newStates
        m_chainCount++
        flag = Solve1Dchain(indices, &tmpIndices, oldState, newStates)
        #flag = Solve1Dchain(indices, NULL, oldState, newStates)
        if newStates.empty():
#ifndef PERFORMANCE_TEST
            std.cout << "Backtracked from level " << m_backTrackLevel << " to level " << m_backTrackLevel - 1 << "not " << std.endl
            ofstream fout
            fout.open(CLevelConfig.AddOutputPrefix("log.txt").c_str(), std.ios_base.app)
            fout << "Backtracked from level " << m_backTrackLevel << " to level " << m_backTrackLevel - 1 << "not " << std.endl
#endif
            m_backTrackCount++
            m_backTrackLevel--

        else:
            m_backTrackLevel++

        if m_ptrGraph.VisitedAllNodes() == True:
            for (i = 0; i < int(newStates.size()); i++)
                if m_solutionCount >= targetNumOfSolutions:
                    break

                SetCurrentState(newStates[i])
                if newStates[i].m_stateEnergy < energyMin:
                    energyMin = newStates[i].m_stateEnergy
                    layoutBest = m_layout

                float collide
                float connectivity
                energy = GetLayoutEnergy(m_layout, m_ptrGraph, collide, connectivity)

                #float CLevelSynth.GetLayoutEnergy(CRoomLayout& layout, ptrGraph, collideArea, connectivity)

                flagValid = (LayoutCollide(m_layout) <= g_numericalTolerance and CheckRoomConnectivity(m_layout, m_ptrGraph) <= g_numericalTolerance)
                if flagValid == False:
                    # Skip invalid solution...
                    continue

                DumpSolutionIntoXML()
                m_solutionCount++


        else:
            for (i = int(newStates.size()) - 1; i >= 0; i--)
                newStates[i].myIndices = indices
                #newStates[i].m_stateGraph = *m_ptrGraph
                graphBest = newStates[i].m_stateGraph
                for (n = 0; n < graphBest.GetNumOfNodes(); n++)
                    pn = newStates[i].m_stateRoomPositions[n]
                    graphBest.GetNode(n).SetPos(pn)


                layoutBest = GetLayout(&graphBest, newStates[i].m_stateRoomPositions)
                #ofstream fout
#ifdef DUMP_PARTIAL_SOLUTION
                graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint("partial_%03d.xml", numPartials)).c_str())
                OpenDoors(layoutBest, &graphBest, True)
                layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("partial_%03d.svg", numPartials)).c_str(), 800, 800, True, &graphBest)
#endif
                #m_bestSolCount ++
                numPartials++

                stateStack.push(newStates[i])



    m_layout = layoutBest
    m_layout.MoveToSceneCenter()

#ifndef PERFORMANCE_TEST
        std.cout << "Total # of backtracks: " << m_backTrackCount << std.endl
        ofstream fout
        fout.open(CLevelConfig.AddOutputPrefix("log.txt").c_str(), std.ios_base.app)
        fout << "Total # of backtracks: " << m_backTrackCount << std.endl

#endif


def Solve1Dchain(self, indices, weightedIndices, oldState, newStates):
    if CLevelConfig.m_flagUseILS == True:
        return Solve1DchainILS(indices, oldState, newStates)

    ptrGraph = &(oldState.m_stateGraph)
    SetSequenceAs1Dchain(indices, ptrGraph)
    newStates.clear()
    if ptrGraph.GetNode(indices[0]).GetFlagFixed() == True:
        oldState.InsertToNewStates(newStates, ptrGraph)
        return True

    flagLastChain = ptrGraph.VisitedAllNodes()

    # Number of cycles
    n = CLevelConfig.m_saNumOfCycles
    # Number of trials per cycle
    m = CLevelConfig.m_saNumOfTrials
    # Number of accepted solutions
    na = 1
    # Probability of accepting worse solution at the start
    p1 = CLevelConfig.m_saProb1
    # Probability of accepting worse solution at the end
    p0 = CLevelConfig.m_saProb0
    # Initial temperature
    t1 = -1.f / log(p1)
    # Final temperature
    t0 = -1.f / log(p0)
    # Fractional reduction every cycle
    frac = pow(t0 / t1, 1.0f / float(n - 1.0f))
    # Current temperature
    t = t1
    # DeltaE Average
    DeltaE_avg = 0.0
    # Current best result so far
    layoutBest = GetLayout(ptrGraph, oldState.m_stateRoomPositions)
    graphBest = *ptrGraph

    if not CLevelConfig.m_flagRandomWalk:
#if 0
		for (i = 0; i < weightedIndices.size(); i++)
			pickedRoom = layoutBest.GetRoom((*weightedIndices)[i])
			SampleConfigSpaceForPickedRoom(layoutBest, ptrGraph, indices, (*weightedIndices)[i])

#else # New on 09/24/2013: connect the rooms together as initial guess
        numOfVisitedNodes = int(indices.size() - weightedIndices.size())
        if (numOfVisitedNodes == 0) # The first chain...
            numOfVisitedNodes = 1

        std.vector<int> indicesVisited(numOfVisitedNodes)
        std.vector<bool> flagsVisited(ptrGraph.GetNumOfNodes(), False)
        for (i = 0; i < numOfVisitedNodes; i++)
            indicesVisited[i] = indices[i]
            flagsVisited[indices[i]] = True

        while (indicesVisited.size() < indices.size())
            idxUnvisited = -1
            idxVisited = -1
            for (i = 0; i < int(weightedIndices.size()); i++)
                idx = (*weightedIndices)[i]
                if flagsVisited[idx] == True:
                    continue

                std.vector<int> connectedIndices = GetConnectedIndices(ptrGraph, idx, False)
                std.vector<int> visitedNeighbors
                for (j = 0; j < int(connectedIndices.size()); j++)
                    idxOther = connectedIndices[j]
                    if flagsVisited[idxOther] == True:
                        visitedNeighbors.push_back(idxOther)


                if visitedNeighbors.empty() == False:
                    idxUnvisited = idx
                    roomUnvisited = layoutBest.GetRoom(idxUnvisited)
                    random_shuffle(visitedNeighbors.begin(), visitedNeighbors.end())
                    CConfigSpace configSpace(layoutBest.GetRoom(visitedNeighbors[0]), roomUnvisited)
                    for (j = 1; j < int(visitedNeighbors.size()); j++)
                        CConfigSpace configSpaceTmp(layoutBest.GetRoom(visitedNeighbors[j]), roomUnvisited)
                        configSpaceNew = CConfigSpace.FindIntersection(configSpace, configSpaceTmp)
                        if configSpaceNew.IsEmpty() == True:
                            break

                        else:
                            configSpace = configSpaceNew


                    indicesVisited.push_back(idxUnvisited)
                    flagsVisited[idxUnvisited] = True
    #if 1 # Smartly sample the configuration space based on the energy...
                    std.vector<v2f> vecPos = configSpace.SmartlySampleConfigSpace()
                    idxBest = -1
                    energyMin = 1e10
                    graphTmp = *ptrGraph
                    for (j = 0; j < graphTmp.GetNumOfNodes(); j++)
                        graphTmp.GetNode(j).SetFlagVisited(flagsVisited[j])

                    for (j = 0; j < int(vecPos.size()); j++)
                        layoutTmp = layoutBest
                        pickedRoomTmp = layoutTmp.GetRoom(idxUnvisited)
                        dp = vecPos[j] - pickedRoomTmp.GetRoomCenter()
                        pickedRoomTmp.TranslateRoom(dp)
                        collideArea = 0.f
                        connectivity = 0.f
                        energyTmp = GetLayoutEnergy(layoutTmp, &graphTmp, collideArea, connectivity)
                        if collideArea < energyMin:
                            energyMin = collideArea
                            idxBest = j


                    if idxBest >= 0:
                        dp = vecPos[idxBest] - roomUnvisited.GetRoomCenter()
                        roomUnvisited.TranslateRoom(dp)

    #else:
                    pos = configSpace.RandomlySampleConfigSpace()
                    dp = pos - roomUnvisited.GetRoomCenter()
                    roomUnvisited.TranslateRoom(dp)
    #endif
                    break



#endif


    m_layout = layoutBest
#ifdef DUMP_INTERMEDIATE_OUTPUT
    layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("chainInit_%03d.svg", m_chainCount)).c_str())
#endif
    collideArea = 0.f
    connectivity = 0.f
    energyMin = GetLayoutEnergy(layoutBest, ptrGraph, collideArea, connectivity, -1, True, &indices)
    energyCurrent = energyMin
#ifdef DUMP_INTERMEDIATE_OUTPUT
    std.cout << "Initial energy: " << energyMin << std.endl
    std.ofstream fout
    fout.open(CLevelConfig.AddOutputPrefix("log.txt").c_str(), std.ios_base.app)
    fout << m_bestSolCount << "\t" << energyMin << std.endl
    for (n = 0; n < ptrGraph.GetNumOfNodes(); n++)
        pn = layoutBest.GetRoomPositions()[n]
        ptrGraph.GetNode(n).SetPos(pn)

    ptrGraph.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint("tmpBest_%03d.xml", m_bestSolCount)).c_str())
    OpenDoors(layoutBest, ptrGraph, True)
    layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("tmpBest_%03d.svg", m_bestSolCount)).c_str())
    m_bestSolCount++
#endif
    pickIndexCount = 0
    numFailures = 0
    for (i = 0; i < n; i++)
#ifndef PERFORMANCE_TEST
        std.cout << "Cycle " << i + 1 << "/" << n << " (failures " << numFailures << ") ...\n"
#endif
        flagWasAccepted = False
        if numFailures > 10:
            if Random2(2) == 0:
#ifndef PERFORMANCE_TEST
                std.cout << "RANDOM RESTART CALLEDnot  11+ failures" << std.endl
#endif
                break


        elif numFailures > 8:
            if Random2(3) == 0:
#ifndef PERFORMANCE_TEST
                std.cout << "RANDOM RESTART CALLEDnot  9+ failures" << std.endl
#endif
                break


        elif numFailures > 5:
            if Random2(4) == 0:
#ifndef PERFORMANCE_TEST
                std.cout << "RANDOM RESTART CALLEDnot  6+ failures" << std.endl
#endif
                break


        elif numFailures > 3:
            if Random2(6) == 0:
#ifndef PERFORMANCE_TEST
                std.cout << "RANDOM RESTART CALLEDnot  4+ failures" << std.endl
#endif
                break


        elif numFailures > 0:
            if Random2(8) == 0:
#ifndef PERFORMANCE_TEST
                std.cout << "RANDOM RESTART CALLED: 1 failure but we just felt like quittin'..." << std.endl
#endif
                break



        for (j = 0; j < m; j++)
            graphTmp = *ptrGraph
            layoutTmp = m_layout
            adjustedIndex = RandomlyAdjustOneRoom(layoutTmp, &graphTmp, indices, weightedIndices)
#if 0 # New on 08/16/2013
			if  m_flagVisitedNoNode == True :
				v2f pMin(1e10)
				v2f pMax(-1e10)
				for ( int d=0; d<int(indices.size()); d++ )
					idx = indices[d]
					pj = layoutTmp.GetRoom(idx).GetRoomCenter()
					for ( int k=0; k<2; k++ )
						pMin[k] = min(pMin[k], pj[k])
						pMax[k] = max(pMax[k], pj[k])


				posCen = (pMin + pMax) * 0.5f
				for ( int d=0; d<int(indices.size()); d++ )
					idx = indices[d]
					layoutTmp.GetRoom(idx).TranslateRoom(-posCen)


#endif
            energyTmp = GetLayoutEnergy(layoutTmp, &graphTmp, collideArea, connectivity, adjustedIndex, True, &indices)

            # accept = GetLayoutEnergyEarlyOut(layoutTmp, &graphTmp, collideArea, connectivity, adjustedIndex, &energyTmp, energyCurrent )

            if ((CLevelConfig.m_flagRandomWalk and collideArea <= 1e-3 and connectivity <= 1e-3) or
                (collideArea <= 1e-4 and connectivity <= 1e-4))
                newState = oldState
                newState.m_stateGraph = graphTmp
                newState.m_stateRoomPositions = layoutTmp.GetRoomPositions()
                newState.m_stateEnergy = energyTmp
                newState.MoveRoomsToSceneCenter(&graphTmp)
                newState.InsertToNewStates(newStates, &graphTmp)
                if newStates.size() >= CLevelConfig.m_numOfSolutionsToTrack:
                    return True

                if flagLastChain == True and (m_solutionCount + int(newStates.size()) >= CLevelConfig.m_targetNumOfSolutions):
                    return True

                #newStates.push_back(newState)

            flagAccept = False

            if energyTmp < energyCurrent:
                # Energy is lower, accept
                flagAccept = True
                if energyTmp < energyMin:
                    energyMin = energyTmp
#ifndef PERFORMANCE_TEST
                    std.cout << "A minimum energy: " << energyMin << std.endl
#endif
#ifdef DUMP_INTERMEDIATE_OUTPUT
                    layoutBest = layoutTmp
                    graphBest = graphTmp
                    for (n = 0; n < graphBest.GetNumOfNodes(); n++)
                        pn = layoutBest.GetRoomPositions()[n]
                        graphBest.GetNode(n).SetPos(pn)


                    std.ofstream fout
                    fout.open(CLevelConfig.AddOutputPrefix("log.txt").c_str(), std.ios_base.app)
                    fout << m_bestSolCount << "\t" << energyMin << std.endl
                    graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint("tmpBest_%03d.xml", m_bestSolCount)).c_str())
                    OpenDoors(layoutBest, &graphBest, True)
                    layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("tmpBest_%03d.svg", m_bestSolCount)).c_str())
                    m_bestSolCount++
#endif


            else:
                DeltaE = std.abs(energyTmp - energyCurrent)

                # Energy is higher...
                if i == 0 and j == 0:
                    DeltaE_avg = DeltaE
                    DeltaE_avg *= CLevelConfig.m_deltaEscaling

                # Generate probability of acceptance...
                prob = exp(-(energyTmp - energyCurrent) / (DeltaE_avg * t))
                r = rand() / float(RAND_MAX)
                if r < prob:
                    flagAccept = True

                else:
                    flagAccept = False


            if flagAccept == True:
                DeltaE = std.abs(energyTmp - energyCurrent)

                m_layout = layoutTmp
                *ptrGraph = graphTmp
                energyCurrent = energyTmp
                if DeltaE != 0.0:
                    na++
                    DeltaE_avg = (DeltaE_avg * (na - 1.0f) + DeltaE) / float(na)

                flagWasAccepted = True


            pickIndexCount++
            pickIndexCount = pickIndexCount % int(indices.size())

        if flagWasAccepted == False:
            numFailures++

        # Lower the temperature for next cycle
        t *= frac

#ifndef PERFORMANCE_TEST
    std.cout << "Final energy: " << energyMin << std.endl
#endif
    if newStates.empty() == True:
#ifdef DUMP_INTERMEDIATE_OUTPUT
        std.cout << "Empty solution setnot \n"
        graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint("backTracking_level%02d_%03d.xml", m_backTrackLevel, m_backTrackCount)).c_str())
        for (n = 0; n < graphBest.GetNumOfNodes(); n++)
            graphBest.GetNode(n).SetFlagVisited(False)

        for (n = 0; n < int(indices.size()); n++)
            graphBest.GetNode(indices[n]).SetFlagVisited(True)

        for (n = 0; n < int(weightedIndices.size()); n++)
            graphBest.GetNode((*weightedIndices)[n]).SetFlagVisited(False)

        layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("backTracking_level%02d_%03d.svg", m_backTrackLevel, m_backTrackCount)).c_str())
        layoutBest.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("backTrackingPartial_level%02d_%03d.svg", m_backTrackLevel, m_backTrackCount)).c_str(), 400, 400, True, &graphBest)
#endif
        return False

#ifndef PERFORMANCE_TEST
    std.cout << "Number of valid states: " << newStates.size() << std.endl
#endif
    sort(newStates.begin(), newStates.end(), CompareStateEnergySmallerFirst)
    numOfSolutionsToTrack = min(int(newStates.size()), CLevelConfig.m_numOfSolutionsToTrack)
    std.vector<CurrentState> newerStates(numOfSolutionsToTrack)
    for (i = 0; i < numOfSolutionsToTrack; i++)
        newerStates[i] = newStates[i]

    newStates = newerStates

    return True


def Solve1DchainILS(self, indices, oldState, newStates):
    ptrGraph = &(oldState.m_stateGraph)
    SetSequenceAs1Dchain(indices, ptrGraph)
    newStates.clear()
    if ptrGraph.GetNode(indices[0]).GetFlagFixed() == True:
        oldState.InsertToNewStates(newStates, ptrGraph)
        return True


    # Borrow the parameters from simulated annealing...
     n = CLevelConfig.m_saNumOfCycles
     m = CLevelConfig.m_saNumOfTrials
    # CRoomLayout layoutBest; # Current best result so far
    collideArea = 0.f
    connectivity = 0.f
    energyMin = 1e10
    energyHistory = 1e10
    pickIndexCount = 0
    for (i = 0; i < n; i++)
        layoutHistory = m_layout
        graphHistory = *ptrGraph
        if i != 0:
            # Introduce perturbation...
            RandomlyAdjustOneRoom(m_layout, ptrGraph, indices, NULL)

        energyTmp = GetLayoutEnergy(m_layout, ptrGraph, collideArea, connectivity)
        energyCurrent = energyTmp
        if i == 0:
            energyMin = energyCurrent
            std.cout << "Initial energy: " << energyCurrent << std.endl

        for (j = 0; j < m; j++)
            graphTmp = *ptrGraph
            layoutTmp = m_layout
            RandomlyAdjustOneRoom(layoutTmp, &graphTmp, indices, NULL)
#if 1 # New on 08/16/2013
            if m_flagVisitedNoNode == True:
                v2f pMin(1e10)
                v2f pMax(-1e10)
                for (d = 0; d < int(indices.size()); d++)
                    idx = indices[d]
                    pj = layoutTmp.GetRoom(idx).GetRoomCenter()
                    for (k = 0; k < 2; k++)
                        pMin[k] = min(pMin[k], pj[k])
                        pMax[k] = max(pMax[k], pj[k])


                posCen = (pMin + pMax) * 0.5f
                for (d = 0; d < int(indices.size()); d++)
                    idx = indices[d]
                    layoutTmp.GetRoom(idx).TranslateRoom(-posCen)


#endif
            energyTmp = GetLayoutEnergy(layoutTmp, &graphTmp, collideArea, connectivity)
            if collideArea <= g_numericalTolerance and connectivity <= g_numericalTolerance:
                newState = oldState
                newState.m_stateGraph = *ptrGraph
                newState.m_stateRoomPositions = layoutTmp.GetRoomPositions()
                newState.m_stateEnergy = energyTmp
                newState.MoveRoomsToSceneCenter(ptrGraph)
                newState.InsertToNewStates(newStates, ptrGraph)

            if energyTmp < energyCurrent:
                if energyTmp < energyMin:
                    # layoutBest = layoutTmp
                    energyMin = energyTmp
#ifndef PERFORMANCE_TEST
                    std.cout << "A minimum energy: " << energyMin << std.endl
#endif

                m_layout = layoutTmp
                *ptrGraph = graphTmp
                energyCurrent = energyTmp

            pickIndexCount++
            pickIndexCount = pickIndexCount % int(indices.size())

        if i == 0 or energyMin < energyHistory:
            energyHistory = energyMin

        else:
            m_layout = layoutHistory
            *ptrGraph = graphHistory


    std.cout << "Final energy: " << energyMin << std.endl
    if newStates.empty() == True:
        std.cout << "Empty solution setnot \n"
        return False

    std.cout << "Number of valid states: " << newStates.size() << std.endl
    sort(newStates.begin(), newStates.end(), CompareStateEnergySmallerFirst)
    numOfSolutionsToTrack = min(int(newStates.size()), CLevelConfig.m_numOfSolutionsToTrack)
    std.vector<CurrentState> newerStates
    for (i = 0; i < numOfSolutionsToTrack; i++)
        newerStates.push_back(newStates[i])

    newStates = newerStates

    return True


def SetCurrentState(self, s):
    *m_ptrGraph = s.m_stateGraph
    m_roomPositions = s.m_stateRoomPositions
    m_layout = GetLayout(m_ptrGraph, m_roomPositions)


def SetSequenceAs1Dchain(self, indices, ptrGraph):
    m_sequence.clear()
    for (i = 0; i < int(indices.size()); i++)
        idx = ptrGraph.GetNode(indices[i]).GetType()
        idx = idx % m_ptrTemplates.GetNumOfTemplates()
        m_sequence.push_back(idx)



def SetVisitedNeighbors(self, indices):
    m_visitedNeighbors.clear()
    m_visitedNeighbors.resize(indices.size())
    for (i = 0; i < int(m_visitedNeighbors.size()); i++)
        nodeIdx = indices[i]
        std.vector<int>neighbors = m_ptrGraph.GetNode(nodeIdx).GetNeighbors()
        for (j = 0; j < int(neighbors.size()); j++)
            neighborIdx = neighbors[j]
            if m_ptrGraph.GetNode(neighborIdx).GetFlagVisited() == True:
                m_visitedNeighbors[i].push_back(neighborIdx)





def DumpSolutionIntoXML(self):
    graphSol = *m_ptrGraph
    for (i = 0; i < graphSol.GetNumOfNodes(); i++)
        pi = m_roomPositions[i]
        graphSol.GetNode(i).SetPos(pi[0], pi[1])

    graphSol.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint("dbg_%03d.xml", m_solutionCount)).c_str())
    layoutSol = GetLayout(m_ptrGraph, m_roomPositions)
    OpenDoors(layoutSol, m_ptrGraph)
    layoutSol.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint("dbg_%03d.svg", m_solutionCount)).c_str())


def RandomlyPickOneRoom(self, layout):
    numOfRooms = layout.GetNumOfRooms()
    pickedRoomIndex = int(rand() / float(RAND_MAX) * numOfRooms)
    pickedRoomIndex = pickedRoomIndex % numOfRooms
    return pickedRoomIndex


def RandomlyPickOneRoom(self, layout, indices, weightedIndices):
    if weightedIndices:
        std.vector<int> tmpIndices = *weightedIndices
        chainLength = int(tmpIndices.size())
        for (i = 0; i < indices.size(); i++)
            energyTmp = layout.GetRoom(indices[i]).GetEnergy()
            if energyTmp > 1.1:
                tmpIndices.push_back(indices[i])



        pickedRoomIndex = int(rand() / float(RAND_MAX) * chainLength)
        pickedRoomIndex = pickedRoomIndex % chainLength
        pickedRoomIndex = (tmpIndices)[pickedRoomIndex]
        return pickedRoomIndex

    else:
        chainLength = int(indices.size())
        pickedRoomIndex = int(rand() / float(RAND_MAX) * chainLength)
        pickedRoomIndex = pickedRoomIndex % chainLength
        pickedRoomIndex = indices[pickedRoomIndex]
        return pickedRoomIndex



def RandomlyPickAnotherRoom(self, layout, pickedIndex):
    numOfRooms = layout.GetNumOfRooms()
    otherRoomIndex = pickedIndex
    while (otherRoomIndex == pickedIndex)
        otherRoomIndex = int(rand() / float(RAND_MAX) * numOfRooms)
        otherRoomIndex = otherRoomIndex % numOfRooms

    return otherRoomIndex


def GetConnectedIndices(self, ptrGraph, pickedIndex, flagVisitedOnly ''' = True '''):
    std.vector<int> indices
    for (i = 0; i < ptrGraph.GetNumOfEdges(); i++)
        edge = ptrGraph.GetEdge(i)
        idx0 = edge.GetIdx0()
        idx1 = edge.GetIdx1()
        if idx0 != pickedIndex and idx1 != pickedIndex:
            continue

        idx = (idx0 == pickedIndex) ? idx1 : idx0
        if ptrGraph.GetNode(idx).GetFlagVisited() == False and flagVisitedOnly == True:
            continue

        indices.push_back(idx)

    return indices


def RandomlyAdjustOneRoom(self, layout, ptrGraph, indices, weightedIndices):
    numOfRooms = layout.GetNumOfRooms()
    if numOfRooms <= 1:
        return -1


    r = rand() / float(RAND_MAX)

#if 0 # Before 07/16/2013
	if  r < 0.25f :
		RandomlyAdjustOneRoom01(layout, ptrGraph, indices)

	elif  r < 0.5f :
		RandomlyAdjustOneRoom02(layout, ptrGraph, indices)

	elif  r < 0.75f or CLevelConfig.m_flagEnableTypeChange == False :
#else:
    if (r < 0.75f or CLevelConfig.m_flagEnableTypeChange == False) # nv: was 0.9
#endif
        if CLevelConfig.m_flagRandomWalk == False:
            return RandomlyAdjustOneRoom03(layout, ptrGraph, indices, weightedIndices)

        else:
            return GradientDescentOneRoom(layout, ptrGraph, *weightedIndices)


    else:
        return RandomlyAdjustOneRoom04(layout, ptrGraph, indices, weightedIndices)



def RandomlyAdjustOneRoom01(self, layout, ptrGraph, indices):
    pickedRoomIndex = RandomlyPickOneRoom(layout, indices, NULL)
    pickedRoom = layout.GetRoom(pickedRoomIndex)

    for (i = 0; i < pickedRoom.GetNumOfEdges(); i++)
        edge = pickedRoom.GetEdge(i)
        pr2 = edge.GetPos2() - edge.GetPos1()
        pr = v3f(pr2[0], pr2[1], 0.f)
        norm = v2f(pr[1], -pr[0])
        norm = normalize(norm)
        distMin = 1e10
        CRoomEdge edgeNearest
        for (j = 0; j < layout.GetNumOfRooms(); j++)
            if j == pickedRoomIndex:
                continue

            if (ptrGraph.GetNode(j).GetFlagVisited() == False) continue
            otherRoom = layout.GetRoom(j)
            for (k = 0; k < otherRoom.GetNumOfEdges(); k++)
                otherEdge = otherRoom.GetEdge(k)
                otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
                otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
                cp = cross(pr, otherPr)
                if mag2(cp) > 0.0001f:
                    continue

                prTmp = otherEdge.GetPos1() - edge.GetPos1()
                distTmp = std.abs(dot(norm, prTmp))
                if distTmp < distMin:
                    distMin = distTmp
                    edgeNearest = otherEdge



        if distMin < 0.3f:
            pr = edgeNearest.GetPos1() - edge.GetPos1()
            d = dot(norm, pr)
            dp = d * norm
            pickedRoom.TranslateRoom(dp)




def RandomlyAdjustOneRoom02(self, layout, ptrGraph, indices):
    pickedRoomIndex = RandomlyPickOneRoom(layout, indices, NULL)
    pickedRoom = layout.GetRoom(pickedRoomIndex)

    numOfEdges = pickedRoom.GetNumOfEdges()
    pickedEdgeIndex = int(rand() / float(RAND_MAX) * numOfEdges)
    pickedEdgeIndex = pickedEdgeIndex % numOfEdges
    edge = pickedRoom.GetEdge(pickedEdgeIndex)
    pr2 = edge.GetPos2() - edge.GetPos1()
    pr = v3f(pr2[0], pr2[1], 0.f)
    norm = v2f(pr[1], -pr[0])
    norm = normalize(norm)

    otherRoomIndex = RandomlyPickAnotherRoom(layout, pickedRoomIndex)
    otherRoom = layout.GetRoom(otherRoomIndex)
    distMin = 1e10
    CRoomEdge edgeNearest

    for (k = 0; k < otherRoom.GetNumOfEdges(); k++)
        otherEdge = otherRoom.GetEdge(k)
        otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
        otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
        cp = cross(pr, otherPr)
        if mag2(cp) > 0.0001f:
            continue

        prTmp = otherEdge.GetPos1() - edge.GetPos1()
        distTmp = std.abs(dot(norm, prTmp))
        if distTmp < distMin:
            distMin = distTmp
            edgeNearest = otherEdge


    if distMin < 0.3f:
        pr = edgeNearest.GetPos1() - edge.GetPos1()
        d = dot(norm, pr)
        dp = d * norm
        pickedRoom.TranslateRoom(dp)



def GradientDescentOneRoom(self, layout, ptrGraph, indices):
    collideArea = 0.f
    connectivity = 0.f
    myEnergy = GetLayoutEnergy(layout, ptrGraph, collideArea, connectivity)
    pickedRoomIndex = RandomlyPickOneRoom(layout, indices, NULL)

    besti = -1
    bestEnergy = 10e6
    std.vector<int> candidateAngles

    for (i = 0; i < 360; i += 10)
        angle = i * atan(1.f) * 4.f / 180.0f
        l = layout
        pickedRoom = l.GetRoom(pickedRoomIndex)

         one_step_length = 0.1f
        dp = v2f(cosf(angle) * one_step_length, sinf(angle) * one_step_length)

        pickedRoom.TranslateRoom(dp)
        collideArea = 0.f
        connectivity = 0.f
        energyMin = GetLayoutEnergy(l, ptrGraph, collideArea, connectivity)
        if energyMin < bestEnergy or besti == -1:
            bestEnergy = energyMin
            besti = i

        if energyMin < myEnergy:
            candidateAngles.push_back(i)



    #	angle = besti * M_PI / 180.0f
    float angle

    if candidateAngles.empty():
        # Wellnot  I'm stuck on a local minimum.
        angle = Random2(36) * 10 * atan(1.f) * 4.f / 180.0f

    else:
        angle = candidateAngles[Random2(candidateAngles.size())] * atan(1.f) * 4.f / 180.0f


    stepSize = 0.1f
    bestStep = -1
    bestEnergy = 10e6

    for (iters = 0; iters < 4; iters++)
        for (stepLength = stepSize; stepLength <= 2; stepLength += stepSize)
            l = layout
            pickedRoom = l.GetRoom(pickedRoomIndex)

            dp = v2f(cosf(angle) * stepLength, sinf(angle) * stepLength)

            pickedRoom.TranslateRoom(dp)
            collideArea = 0.f
            connectivity = 0.f
            energyMin = GetLayoutEnergy(l, ptrGraph, collideArea, connectivity)
            if energyMin < bestEnergy or bestStep == -1:
                bestEnergy = energyMin
                bestStep = stepLength


        if myEnergy - bestEnergy < 0.00001:
            stepSize /= 2.0f



    dp = v2f(cosf(angle) * bestStep, sinf(angle) * bestStep)

    # do the actual translation on the room itself
    pickedRoom = layout.GetRoom(pickedRoomIndex)
    pickedRoom.TranslateRoom(dp)
    return pickedRoomIndex


def RandomlyAdjustOneRoom03(self, layout, ptrGraph, indices, weightedIndices):
    pickedRoomIndex = RandomlyPickOneRoom(layout, indices, weightedIndices)
    pickedRoom = layout.GetRoom(pickedRoomIndex)

    SampleConfigSpaceForPickedRoom(layout, ptrGraph, indices, pickedRoomIndex)
    return pickedRoomIndex


def SampleConfigSpaceForPickedRoom(self, layout, ptrGraph, indices, pickedRoomIndex):
    pickedRoom = layout.GetRoom(pickedRoomIndex)
    CConfigSpace configSpace
    std.vector<int> connectedIndices = GetConnectedIndices(ptrGraph, pickedRoomIndex)
    if connectedIndices.size() >= 1:
        random_shuffle(connectedIndices.begin(), connectedIndices.end())
        idx0 = connectedIndices[0]
        CConfigSpace configSpace0(layout.GetRoom(idx0), pickedRoom)
        configSpace = configSpace0
        for (i = 1; i < int(connectedIndices.size()); i++)
            CConfigSpace configSpaceTmp(layout.GetRoom(connectedIndices[i]), pickedRoom)
            configSpaceNew = CConfigSpace.FindIntersection(configSpace, configSpaceTmp)
            if configSpaceNew.IsEmpty() == True:
                break

            else:
                configSpace = configSpaceNew



    whileCnt = 0
    while (configSpace.IsEmpty() == True)
        otherRoomIndex = RandomlyPickAnotherRoom(layout, pickedRoomIndex)
        otherRoom = layout.GetRoom(otherRoomIndex)
        configSpace = CConfigSpace(otherRoom, pickedRoom)
        whileCnt++
        if whileCnt >= 1000:
            std.cout << "Break from the while loop after reaching enough number of trialsnot \n"
            return


    pos = configSpace.RandomlySampleConfigSpace()
    dp = pos - pickedRoom.GetRoomCenter()
    pickedRoom.TranslateRoom(dp)


def RandomlyAdjustOneRoom04(self, layout, ptrGraph, indices, weightedIndices):
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    if numOfTemplates <= 1:
        return -1


    pickedRoomIndex = RandomlyPickOneRoom(layout, indices, weightedIndices)
    pickedRoom = layout.GetRoom(pickedRoomIndex)

    typeOld = ptrGraph.GetNode(pickedRoomIndex).GetType()
    typeNew = typeOld
    boundaryOld = ptrGraph.GetNode(pickedRoomIndex).GetBoundaryType()
    boundaryNew = -1
    whileCnt = 0
    while (typeNew == typeOld or boundaryNew != boundaryOld or m_ptrTemplates.GetRoom(typeNew).GetBoundaryType() == 1)
        typeNew = int(rand() / float(RAND_MAX) * numOfTemplates)
        typeNew = typeNew % numOfTemplates
        boundaryNew = m_ptrTemplates.GetRoom(typeNew).GetBoundaryType()
        whileCnt++
        if whileCnt >= 1000:
            std.cout << "Break from the while loop after reaching enough number of trials in RandomlyAdjustOneRoom04()not \n"
            return -1


    ptrGraph.GetNode(pickedRoomIndex).SetType(typeNew)
    room = m_ptrTemplates.GetRoom(typeNew)
    p1 = room.GetRoomCenter()
    p2 = pickedRoom.GetRoomCenter()
    dp = p2 - p1
    room.TranslateRoom(dp)
    pickedRoom = room
#if 1 # New on 09/15/2013
    #SampleConfigSpaceForPickedRoom(layout, ptrGraph, indices, pickedRoomIndex)
#endif
    return pickedRoomIndex


def GetLayoutEnergyEarlyOut(self, layout, ptrGraph, collideArea, connectivity, roomMoved, energyTmp, energyCurrent):
    layout.ResetRoomEnergies()
    *energyTmp = 1.f

    # do connectivity first, it's (probably?) cheaper

    if CLevelConfig.m_sigmaConnectivity > 0.f:
        connectivity = CheckRoomConnectivity(layout, ptrGraph, True, roomMoved)
        (*energyTmp) *= exp(connectivity * CLevelConfig.m_sigmaConnectivity)

    if *energyTmp > energyCurrent:
        return False

    if CLevelConfig.m_sigmaCollide > 0.f:
        collideArea = LayoutCollide(layout, ptrGraph, True, roomMoved)
        (*energyTmp) *= exp(collideArea * CLevelConfig.m_sigmaCollide)

    if *energyTmp > energyCurrent:
        return False


    if CLevelConfig.m_sigmaContact > 0.f:
        contactArea = LayoutContact(layout, ptrGraph, True, CLevelConfig.m_flagNonOverlapContact)
        (*energyTmp) *= exp(-contactArea * CLevelConfig.m_sigmaContact)

    return True


def GetLayoutEnergy(self, layout, ptrGraph, collideArea, connectivity, roomMoved, doContact, indices):
    layout.ResetRoomEnergies()
    layoutEnergy = 1.f
    if CLevelConfig.m_sigmaCollide > 0.f:
        collideArea = LayoutCollide(layout, ptrGraph, True, roomMoved)
        layoutEnergy *= exp(collideArea * CLevelConfig.m_sigmaCollide)


    if CLevelConfig.m_sigmaConnectivity > 0.f:
        connectivity = CheckRoomConnectivity(layout, ptrGraph, True, roomMoved)
        layoutEnergy *= exp(connectivity * CLevelConfig.m_sigmaConnectivity)

    if CLevelConfig.m_sigmaContact > 0.f and doContact:
        contactArea = -LayoutContact(layout, ptrGraph, True, CLevelConfig.m_flagNonOverlapContact, indices)

        if contactArea >= 0.0f:
            contactArea = 0.0

        if contactArea < 0:
            layoutEnergy *= exp(contactArea / CLevelConfig.m_sigmaContact)



    return layoutEnergy


def CheckRoomConnectivity(self, layout, ptrGraph, flagVisitedOnly ''' = False ''', roomMoved):
    connectivity = 0.f
    if ptrGraph == NULL:
        return connectivity

    for (i = 0; i < ptrGraph.GetNumOfEdges(); i++)
        edge = ptrGraph.GetEdge(i)
        idx0 = edge.GetIdx0()
        idx1 = edge.GetIdx1()
        flagVisited0 = ptrGraph.GetNode(idx0).GetFlagVisited()
        flagVisited1 = ptrGraph.GetNode(idx1).GetFlagVisited()
        if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
            continue

        flagFixed0 = ptrGraph.GetNode(idx0).GetFlagFixed()
        flagFixed1 = ptrGraph.GetNode(idx1).GetFlagFixed()
        if flagFixed0 == True and flagFixed1 == True:
            continue

        if roomMoved == -1 or roomMoved == idx0 or roomMoved == idx1 or layout.cachedConnectivities.find(std.make_pair(idx0, idx1)) == layout.cachedConnectivities.end():
            contactArea = RoomContact(layout.GetRoom(idx0), layout.GetRoom(idx1))
            if contactArea <= CLevelConfig.m_roomContactThresh:
                if CLevelConfig.m_flagDiscreteConnectFunc == True:
                    connectivity += 1.f
                    layout.cachedConnectivities[std.make_pair(idx0, idx1)] = 1.f

                else:
                    d = RoomDistance(layout.GetRoom(idx0), layout.GetRoom(idx1))
                    d += CLevelConfig.m_roomContactThresh
                    layout.cachedConnectivities[std.make_pair(idx0, idx1)] = d
                    connectivity += d

                factor = 1.1f
                layout.GetRoom(idx0).UpdateEnergy(factor)
                layout.GetRoom(idx1).UpdateEnergy(factor)

            else:
                layout.cachedConnectivities[std.make_pair(idx0, idx1)] = 0.0f


        else:
            connectivity += layout.cachedConnectivities[std.make_pair(idx0, idx1)]



    return connectivity


def LayoutCollide(self, layout, ptrGraph, flagVisitedOnly ''' = False ''', roomThatMoved ''' = -1 '''):
    collideAreaTotal = 0.f
    collideCount = 0
    numOfRooms = layout.GetNumOfRooms()
    for (i = 0; i < numOfRooms; i++)
        for (j = i + 1; j < numOfRooms; j++)
            flagVisited0 = ptrGraph.GetNode(i).GetFlagVisited()
            flagVisited1 = ptrGraph.GetNode(j).GetFlagVisited()
            if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
                continue

            flagFixed0 = ptrGraph.GetNode(i).GetFlagFixed()
            flagFixed1 = ptrGraph.GetNode(j).GetFlagFixed()
            if flagFixed0 == True and flagFixed1 == True:
                continue

            if roomThatMoved == -1 or roomThatMoved == i or roomThatMoved == j or layout.cachedCollisionEnergies.find(std.make_pair(i, j)) == layout.cachedCollisionEnergies.end():
                collideArea = RoomCollides(layout.GetRoom(i), layout.GetRoom(j))
                if collideArea > 0.f:
                    collideAreaTotal += collideArea
                    collideCount++
                    factor = exp(collideArea)
                    layout.GetRoom(i).UpdateEnergy(factor)
                    layout.GetRoom(j).UpdateEnergy(factor)
                    layout.cachedCollisionEnergies[std.make_pair(i, j)] = collideArea

                else:
                    layout.cachedCollisionEnergies[std.make_pair(i, j)] = collideArea


            else:
                collideAreaTotal += layout.cachedCollisionEnergies[std.make_pair(i, j)]




#ifdef PRINT_OUT_DEBUG_INFO
    std.cout << "Number of colliding room pairs: " << collideCount << std.endl
    std.cout << "Total area of colliding area: " << collideAreaTotal << std.endl
#endif
    return collideAreaTotal


def LayoutCollide(self, layout):
    collideAreaTotal = 0.f
    collideCount = 0
    numOfRooms = layout.GetNumOfRooms()
    for (i = 0; i < numOfRooms; i++)
        for (j = i + 1; j < numOfRooms; j++)
            if layout.GetRoom(i).GetBoundaryType() == 1 and layout.GetRoom(j).GetBoundaryType() == 1:
                continue

            collideArea = RoomCollides(layout.GetRoom(i), layout.GetRoom(j))
            if collideArea > 0.f:
                collideAreaTotal += collideArea
                collideCount++



#ifdef PRINT_OUT_DEBUG_INFO
    std.cout << "Number of colliding room pairs: " << collideCount << std.endl
    std.cout << "Total area of colliding area: " << collideAreaTotal << std.endl
#endif
    return collideAreaTotal


def RoomCollides(self, room1, room2):
    collideArea = -1.f

    # Test the bounding box first...
    AABB2f bb1, bb2
    room1.GetRoomBoundingBox(bb1)
    room2.GetRoomBoundingBox(bb2)
    if TestBoundingBoxCollides(bb1, bb2) == False:
        return 0.f

    # Use the Clipper library...
    CClipperWrapper wrapper
    collideArea = wrapper.ComputeCollideArea(room1, room2)

    return collideArea


def BoundingBoxCollidesArea(self, bb1, bb2):
    collideArea = -1.f
    for (j = 0; j < 2; j++)
        if bb1.m_posMax[j] < bb2.m_posMin[j] or bb1.m_posMin[j] > bb2.m_posMax[j]:
            return collideArea


    collideArea = 1.f
    for (j = 0; j < 2; j++)
        pMin = max(bb1.m_posMin[j], bb2.m_posMin[j])
        pMax = min(bb1.m_posMax[j], bb2.m_posMax[j])
        if (pMin > pMax) return -1.f
        pd = pMax - pMin
        collideArea *= pd

    return collideArea


def TestBoundingBoxCollides(self, bb1, bb2):
    for (j = 0; j < 2; j++)
        if bb1.m_posMax[j] < bb2.m_posMin[j] or bb1.m_posMin[j] > bb2.m_posMax[j]:
            return False


    return True


def LayoutContact(self, layout, ptrGraph, flagVisitedOnly ''' = False ''', flagNonOverlap ''' = False ''', indices, roomThatMoved ''' probably == null '''):
    contactAreaTotal = 0.f
    contactCount = 0
    numOfRooms = layout.GetNumOfRooms()

    for (i = 0; i < numOfRooms; i++)
        flagVisited0 = ptrGraph.GetNode(i).GetFlagVisited()
        if flagVisited0 == False:
            continue


        badNeighbour = False
        std.vector<int>neighbours = ptrGraph.GetNode(i).GetNeighbors()
        for (j = 0; j < neighbours.size(); j++)
            found = False
            for (k = 0; k < indices.size(); k++)
                if (*indices)[k] == neighbours[j]:
                    found = True
                    break


            if not found:
                badNeighbour = True
                break


        if not badNeighbour:
            continue

        perimeter = RoomPerimeter(layout.GetRoom(i))

        for (j = i + 1; j < numOfRooms; j++)
            flagVisited1 = ptrGraph.GetNode(j).GetFlagVisited()
            if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
                continue

            if i == roomThatMoved or j == roomThatMoved or roomThatMoved == -1 or layout.cachedContacts.find(std.make_pair(i, j)) == layout.cachedContacts.end():
                if RoomCollides(layout.GetRoom(i), layout.GetRoom(j)) > 0.f:
                    layout.cachedContacts[std.make_pair(i, j)] = 0.0f
                    continue

                contactArea = RoomContact(layout.GetRoom(i), layout.GetRoom(j))
                if (contactArea > CLevelConfig.m_roomContactThresh) #0.f
                    contactArea -= CLevelConfig.m_roomContactThresh
                    layout.cachedContacts[std.make_pair(i, j)] = contactArea
                    perimeter -= contactArea
                    contactCount++


            else:
                perimeter -= layout.cachedContacts[std.make_pair(i, j)]


        if perimeter > 0:
            contactAreaTotal += perimeter


#ifdef PRINT_OUT_DEBUG_INFO
    std.cout << "Number of contacting room pairs: " << contactCount << std.endl
    std.cout << "Total area of contacting area: " << contactAreaTotal << std.endl
#endif
    return contactAreaTotal


def ComputeLabelPosition(self, idx, ptrGraph, labelRad):
    v2f pMin, pMax
    ptrGraph.GetGraphBoundingBox(pMin, pMax)
    vMax = max(max(std.abs(pMin[0]), std.abs(pMin[1])), max(std.abs(pMax[0]), std.abs(pMax[1])))
     pos = ptrGraph.GetNodePos(idx)
     rad = labelRad
    n = 32
    dMinMax = -1e10
    v2f piMax
    for (i = 0; i < n; i++)
        angle = atan(1.f) * 8.f * float(i) / float(n)
        cv = -cos(angle) * rad
        sv = sin(angle) * rad
        pi = pos + v2f(cv, sv)
        if pi[0] < -vMax or pi[0] > vMax or pi[1] < -vMax or pi[1] > vMax:
            continue

        dMin = 1e10
        for (j = 0; j < ptrGraph.GetNumOfEdges(); j++)
            edge = ptrGraph.GetEdge(j)
            idx1 = edge.GetIdx0()
            idx2 = edge.GetIdx1()
            if idx1 != idx and idx2 != idx:
                continue

            pos1 = ptrGraph.GetNodePos(idx1)
            pos2 = ptrGraph.GetNodePos(idx2)
            dTmp = PointToSegmentSqDistance(pi, CLineBase(pos1, pos2))
            if dTmp < dMin:
                dMin = dTmp


        if dMin >= rad * rad:
            return pi

        if dMin > dMinMax:
            dMinMax = dMin
            piMax = pi


    return piMax

