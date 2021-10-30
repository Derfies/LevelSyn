class CurrentState:

    def __init__(self):
        # CPlanarGraph state_graph
        # std.vector<v2f> state_room_positions
        # std.vector<int> my_indices
        # float state_energy
        pass

    def move_rooms_to_scene_center(self, graph):
        pass

    def move_1d_chain_to_scene_center(self, indices):
        pass

    def get_state_difference(self, otherState, graph):
        pass

    def insert_to_new_states(self, new_states, graph):
        pass


def Random2(self, max):
    if max < 1 or max >= RAND_MAX:
        return 0
    else:
        return (int)rand() / (RAND_MAX / max + 1)


def move_rooms_to_scene_center(self, graph):
    v2f p_min(1e10)
    v2f p_max(-1e10)
    for (j = 0; j < graph.GetNumOfNodes(); j++):
        if graph.GetNode(j).GetFlagVisited() == False:
            continue

        pj = state_room_positions[j]
        for (k = 0; k < 2; k++):
            p_min[k] = min(p_min[k], pj[k])
            p_max[k] = max(p_max[k], pj[k])


    pos_cen = (p_min + p_max) * 0.5f
    for (j = 0; j < graph.GetNumOfNodes(); j++):
        state_room_positions[j] = state_room_positions[j] - pos_cen



def move_1d_chain_to_scene_center(self, indices):
    v2f p_min(1e10)
    v2f p_max(-1e10)
    for (j = 0; j < int(indices.size()); j++):
        idx = indices[j]
        pj = state_room_positions[idx]
        for (k = 0; k < 2; k++):
            p_min[k] = min(p_min[k], pj[k])
            p_max[k] = max(p_max[k], pj[k])


    pos_cen = (p_min + p_max) * 0.5f
    for (j = 0; j < int(indices.size()); j++):
        idx = indices[j]
        state_room_positions[idx] = state_room_positions[idx] - pos_cen



def get_state_difference(self, otherState, graph):
    stateDiff = 0.f
    for (j = 0; j < graph.GetNumOfNodes(); j++):
        # graph.GetNode(picked_room_index).SetType(typeNew)

        if graph.GetNode(j).GetFlagVisited() == False:
            continue

        '''
        if graph.GetNode(j).GetType() != otherState.state_graph.GetNode(j).GetType():
        stateDiff += 2 * CLevelConfig.m_stateDiffThresh

        '''
        p1 = state_room_positions[j]
        p2 = otherState.state_room_positions[j]
        stateDiff += mag2(p1 - p2)

    return stateDiff


def insert_to_new_states(self, new_states, graph):
    stateDiffThresh = CLevelConfig.m_stateDiffThresh
    if stateDiffThresh <= 0.f:
        new_states.push_back(*self)
        return True

    for (i = 0; i < int(new_states.size()); i++):
        if state_energy < new_states[i].state_energy:
            continue

        stateDiff = get_state_difference(new_states[i], graph)
        if stateDiff <= stateDiffThresh:
            return False

    new_states.push_back(*self)
    return True


CLevelSynth.CLevelSynth()
    self.graph = NULL
    m_ptrTemplates = NULL
    self.solution_count = 0
    m_bestSolCount = 0
    self.chain_count = 0
    self.backtrack_count = 0
    self.backtrack_level = 0


CLevelSynth.CLevelSynth(CPlanarGraph* graph, ptrTemplates)
    SetGraphAndTemplates(graph, ptrTemplates)


def SetGraphAndTemplates(self, graph, ptrTemplates):
    self.solution_count = 0
    m_bestSolCount = 0
    graph.MoveGraphToSceneCenter()
    graph.ScaleGraphNodePositions(CLevelConfig.m_graphScaling)
    SetGraph(graph)
    m_ptrTemplates = ptrTemplates
    self.graph.SetNumOfTypes(m_ptrTemplates.GetNumOfTemplates())
    self.graph.RandomInitTypes()

    #self.graph.RandomInitPositions()
    InitScene()
    SynthesizeScene()


def SetGraph(self, graph):
    self.graph = graph
    room_positions.resize(self.graph.GetNumOfNodes())
    for (i = 0; i < self.graph.GetNumOfNodes(); i++):
        pi = self.graph.GetNodePos(i)
        room_positions[i] = pi



def MovePickedGraphNode(self, dx, dy):
    self.graph.MovePickedNode(dx, dy)
    InitScene()
    flag = False
    #flag = AdjustPickedRoom(dx, dy)
    return flag


def AdjustPickedRoom(self, dx, dy):
    pickedNodeIndex = self.graph.GetPickedNodeIndex()
    flag = False
    if pickedNodeIndex < 0:
        return flag

    pickedRoom = self.layout.get_room(pickedNodeIndex)
    for (i = 0; i < pickedRoom.GetNumOfEdges(); i++):
        edge = pickedRoom.GetEdge(i)
        pr2 = edge.GetPos2() - edge.GetPos1()
        pr = v3f(pr2[0], pr2[1], 0.f)
        norm = v2f(pr[1], -pr[0])
        norm = normalize(norm)
        distMin = 1e10
        CRoomEdge edgeNearest
        for (j = 0; j < self.layout.GetNumOfRooms(); j++):
            if j == pickedNodeIndex:
                continue

            otherRoom = self.layout.get_room(j)
            for (k = 0; k < otherRoom.GetNumOfEdges(); k++):
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
            self.graph.MovePickedNode(dp)
            InitScene()
            dx += dp[0]
            dy += dp[1]
            flag = True


    return flag


def InitScene(self):
    self.layout.ClearLayout()
    numOfRooms = self.graph.GetNumOfNodes()
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    for (i = 0; i < numOfRooms; i++):
        #idx = int(rand() / float(RAND_MAX) * numOfTemplates)
        idx = self.graph.GetNode(i).GetType()
        idx = idx % numOfTemplates
        room = m_ptrTemplates.get_room(idx)
        #room.ScaleRoom(0.5f)
        pi = self.graph.GetNodePos(i)
        c = room.get_room_centre()
        trans = pi - c
        room.translate_room(trans)
        color = randomColorFromIndex(i)
        if mag2(color) > 2.5f:
            color = color * 0.5f

        room.SetColor(color)
        self.layout.AddRoom(room)



def get_layout(self, graph, room_positions):
    CRoomLayout layout
    numOfRooms = graph.GetNumOfNodes()
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    for (i = 0; i < numOfRooms; i++):
        idx = graph.GetNode(i).GetType()
        idx = idx % numOfTemplates
        room = m_ptrTemplates.get_room(idx)
        pi = room_positions[i]
        c = room.get_room_centre()
        trans = pi - c
        room.translate_room(trans)
        color = randomColorFromIndex(i)
        if mag2(color) > 2.5f:
            color = color * 0.5f

        room.SetColor(color)
        room.SetFlagFixed(graph.GetNode(i).GetFlagFixed())
        layout.AddRoom(room)

    return layout


def SynthesizeScene(self):
    synthesize_scene_via_main_loop()
    UpdateGraphFromLayout()


def UpdateGraphFromLayout(self):
    numOfRooms = self.graph.GetNumOfNodes()
    for (i = 0; i < numOfRooms; i++):
        roomCenter = self.layout.get_room(i).get_room_centre()
        self.graph.GetNode(i).SetPos(roomCenter)



def PostProcessing(self, layout, graph):
    for (i = 0; i < layout.GetNumOfRooms(); i++):
        std.vector<int> neighbors
        for (j = 0; j < graph.GetNumOfEdges(); j++):
            edge = graph.GetEdge(j)
            if edge.GetIdx0() == i:
                neighbors.push_back(edge.GetIdx1())

            elif edge.GetIdx1() == i:
                neighbors.push_back(edge.GetIdx0())


        if neighbors.empty() == True:
            continue

        CConfigSpace configSpace0(layout.get_room(neighbors[0]), layout.get_room(i))
        for (j = 1; j < int(neighbors.size()); j++):
            CConfigSpace configSpace1(layout.get_room(neighbors[j]), layout.get_room(i))
            configSpace0 = CConfigSpace.FindIntersection(configSpace0, configSpace1)

        configSpace0.SelfMerge()
        print(f'Size of configuration space for the {i}th room: {configSpace0.GetConfigSpaceSize()}')
        configSpace0.PrintConfigSpace()

    return True


def OpenDoors(self, layout, graph, flag_partial ''' = False '''):
    for (i = 0; i < layout.GetNumOfRooms(); i++):
        layout.get_room(i).InitWalls()

    for (i = 0; i < graph.GetNumOfEdges(); i++):
        ge = graph.GetEdge(i)
        roomIdx1 = ge.GetIdx0()
        roomIdx2 = ge.GetIdx1()
        room1 = layout.get_room(roomIdx1)
        room2 = layout.get_room(roomIdx2)
        if room1.GetFlagFixed() or room2.GetFlagFixed():
            continue

        int edgeIdx1, edgeIdx2
        contact = RoomContact(room1, room2, edgeIdx1, edgeIdx2)
        if (contact < CLevelConfig.m_roomContactThresh): # just to double check
            if not flag_partial:
                print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 1)!')
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
        if not flag1 or not flag2:
            print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 2)!')
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


def OpenDoors(self, layout, layoutShrinked, graph, thrinkDist):
    for (i = 0; i < layout.GetNumOfRooms(); i++):
        layout.get_room(i).InitWalls()
        layoutShrinked.get_room(i).InitWalls()

    doorWidth = CLevelConfig.m_roomContactThresh * 0.8f
    for (i = 0; i < graph.GetNumOfEdges(); i++):
        ge = graph.GetEdge(i)
        roomIdx1 = ge.GetIdx0()
        roomIdx2 = ge.GetIdx1()
        room1 = layout.get_room(roomIdx1)
        room2 = layout.get_room(roomIdx2)
        if room1.GetFlagFixed() == True or room2.GetFlagFixed() == True:
            continue

        int edgeIdx1, edgeIdx2
        contact = RoomContact(room1, room2, edgeIdx1, edgeIdx2)
        if (contact < doorWidth) # just to double check
            print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 1)!')
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
        flag1 = OpenDoor(layoutShrinked.get_room(roomIdx1), door1, doorWidth) or OpenDoor(layoutShrinked.get_room(roomIdx1), door2, doorWidth)
        flag2 = OpenDoor(layoutShrinked.get_room(roomIdx2), door1, doorWidth) or OpenDoor(layoutShrinked.get_room(roomIdx2), door2, doorWidth)
        if flag1 == False or flag2 == False:
            print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 2)!')
            return False

    return True

def ShrinkRooms(self, layout, dist):
    if dist <= 0.f:
        return

    for (i = 0; i < layout.GetNumOfRooms(); i++):
        ShrinkRoom(layout.get_room(i), dist)

def ShrinkRoom(self, room, dist):
    if dist <= 0.f:
        return

     numOfEdges = room.GetNumOfEdges()
    std.vector<CRoomEdge> vecRoomEdge(numOfEdges)
    for (i = 0; i < numOfEdges; i++):
        vecRoomEdge[i] = room.GetEdge(i)

    std.vector<CRoomEdge> vecRoomEdge1 = vecRoomEdge
    std.vector<CRoomEdge> vecRoomEdge2 = vecRoomEdge
    for (i = 0; i < numOfEdges; i++):
        pr = vecRoomEdge[i].GetPos2() - vecRoomEdge[i].GetPos1()
        prNew = v2f(pr[1], -pr[0])
        prNew = normalize(prNew) * dist
        vecRoomEdge1[i].SetPos1(vecRoomEdge1[i].GetPos1() + prNew)
        vecRoomEdge1[i].SetPos2(vecRoomEdge1[i].GetPos2() + prNew)
        vecRoomEdge2[i].SetPos1(vecRoomEdge2[i].GetPos1() - prNew)
        vecRoomEdge2[i].SetPos2(vecRoomEdge2[i].GetPos2() - prNew)

    std.vector<v2f> vertices1 = room.GetVertices()
    std.vector<v2f> vertices2 = room.GetVertices()
    for (i = 0; i < numOfEdges; i++):
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


def SaveGraphAsSVG(self, fileName, graph, wd ''' = 400 ''', ht ''' = 400 ''', labelRad ''' = 0.25f '''):
    graphNew = *graph
    graphNew.MoveGraphToSceneCenter()
    strokeWd = 5
    circleRad = 7
    v2f posMin, posMax
    graphNew.GetGraphBoundingBox(posMin, posMax)
    p_min = min(posMin[0], posMin[1])
    p_max = max(posMax[0], posMax[1])
    scaling = 1.05f
    p_min *= scaling
    p_max *= scaling
     str = '\t<?xml version=\'1.0\' standalone=\'no\' ?>\n'
                      '<not -- graph visualization -.\n'
                      '<svg>\n'
                      '</svg>\n'
    tinyxml2.XMLDocument doc
    doc.Parse(str)
    root = doc.RootElement()
    std.ostringstream ossViewBox
    ossViewBox << 0} {0} {wd} {ht
    root.SetAttribute('viewBox', ossViewBox.str().c_str())
    root.SetAttribute('xmlns', 'http:#www.w3.org/2000/svg')
    # Dump edges...
    for (i = 0; i < graphNew.GetNumOfEdges(); i++):
        edgeElement = doc.NewElement('path')
        std.ostringstream ossPath
        edge = graphNew.GetEdge(i)
        p1 = graphNew.GetNodePos(edge.GetIdx0())
        p2 = graphNew.GetNodePos(edge.GetIdx1())
        ossPath}M '
        ossPath << CRoomLayout.ConvertPosX(p1[0], p_min, p_max, wd)} '
        ossPath << CRoomLayout.ConvertPosY(p1[1], p_min, p_max, ht)} '
        ossPath}L '
        ossPath << CRoomLayout.ConvertPosX(p2[0], p_min, p_max, wd)} '
        ossPath << CRoomLayout.ConvertPosY(p2[1], p_min, p_max, ht)} '
        edgeElement.SetAttribute('d', ossPath.str().c_str())
        edgeElement.SetAttribute('fill', 'none')
        edgeElement.SetAttribute('stroke', 'black')
        edgeElement.SetAttribute('stroke-width', strokeWd)
        root.InsertEndChild(edgeElement)

    # Dump nodes...
    for (i = 0; i < graphNew.GetNumOfNodes(); i++):
        nodeElement = doc.NewElement('circle')
        pi = graphNew.GetNodePos(i)
        nodeElement.SetAttribute('cx', CRoomLayout.ConvertPosX(pi[0], p_min, p_max, wd))
        nodeElement.SetAttribute('cy', CRoomLayout.ConvertPosY(pi[1], p_min, p_max, ht))
        nodeElement.SetAttribute('r', circleRad)
        nodeElement.SetAttribute('fill', 'red')
        nodeElement.SetAttribute('stroke', 'none')
        root.InsertEndChild(nodeElement)

    # Dump labels...
    for (i = 0; i < graphNew.GetNumOfNodes(); i++):
        shiftX = (i >= 10) ? 8 : 3
        shiftY = 5
        pi = ComputeLabelPosition(i, &graphNew, labelRad)
        labelElement = doc.NewElement('text')
        labelElement.SetAttribute('x', CRoomLayout.ConvertPosX(pi[0], p_min, p_max, wd) - shiftX)
        labelElement.SetAttribute('y', CRoomLayout.ConvertPosY(pi[1], p_min, p_max, ht) + shiftY)
        labelElement.SetAttribute('font-family', 'Verdana')
        labelElement.SetAttribute('font-size', 13)
        labelElement.SetAttribute('fill', 'blue')
        std.ostringstream ossLabel
        ossLabel << i
        labelText = doc.NewText(ossLabel.str().c_str())
        labelElement.InsertEndChild(labelText)
        root.InsertEndChild(labelElement)


    saveFlag = doc.SaveFile(fileName)
    return saveFlag


def CompareStateEnergySmallerFirst(self, state1, state2):
    return (state1.state_energy < state2.state_energy)


def synthesize_scene_via_main_loop(self):
    state0 = CurrentState()
    state0.state_graph = self.graph
    state0.state_room_positions = self.room_positions
    state0.state_energy = 1e10
    state_stack = []
    state_stack.insert(0, state0)
    target_num_solutions = TARGET_NUM_SOLUTIONS
    energy_min = 1e10
    layout_best = self.layout
    num_partials = 0
    self.backtrack_count = 0
    self.backtrack_level = 0
    while self.solution_count < target_num_solutions and state_stack:
        old_state = state_stack.pop(0)
        self.set_current_state(old_state)
        self.flag_visited_node = self.graph.VisitedNoNode()
        flag_cyclic = False
        tmp_indices = self.graph.ExtractDeepestFaceOrChain(flag_cyclic, FLAG_SMALL_FACE_FIRST)
        indices = []
#if 0 # Before 09/03/2013
        if SYN_METHOD != 0 :
            # Select all the graph nodes...
            indices.resize(self.graph.GetNumOfNodes())
            for i in range(len(indices)):
                indices[i] = i
#else:
        indices = old_state.my_indices
        if SYN_METHOD != 0:
            if not self.graph.HasFixedNode() or not self.graph.VisitedNoNode():
                indices = self.graph.GetUnfixedNodes()

        for i in range(len(tmp_indices)):
            indices.append(tmp_indices[i])
#endif
        SetVisitedNeighbors(indices)
        for (i = 0; i < self.graph.GetNumOfNodes(); i++):
            self.graph.GetNode(i).SetFlagVisited(False)

        for (i = 0; i < int(indices.size()); i++):
            index = indices[i]
            self.graph.GetNode(indices[i]).SetFlagVisited(True)

        old_state.state_graph = self.graph
        new_states = []
        self.chain_count += 1
        flag = self.solve_1d_chain(indices, tmp_indices, old_state, new_states)
        if not new_states:
#ifndef PERFORMANCE_TEST
            print(f'Backtracked from level {self.backtrack_level} to level {self.backtrack_level - 1}!')
#endif
            self.backtrack_count += 1
            self.backtrack_level -= 1
        else:
            self.backtrack_level += 1

        if self.graph.VisitedAllNodes():
            for i in range(len(new_states)):
                if self.solution_count >= target_num_solutions:
                    break

                self.set_current_state(new_states[i])
                if new_states[i].state_energy < energy_min:
                    energy_min = new_states[i].state_energy
                    layout_best = self.layout

                #float collide
                #float connectivity
                energy = self.get_layout_energy(self.layout, self.graph, collide, connectivity)

                #float CLevelSynth.get_layout_energy(CRoomLayout& layout, graph, collide_area, connectivity)

                flag_valid = LayoutCollide(self.layout) <= g_numericalTolerance and CheckRoomConnectivity(self.layout, self.graph) <= g_numericalTolerance
                if not flag_valid:
                    # Skip invalid solution...
                    continue

                DumpSolutionIntoXML()
                self.solution_count += 1

        else:
            for (i = int(new_states.size()) - 1; i >= 0; i--):
                new_states[i].my_indices = indices
                #new_states[i].state_graph = *self.graph
                graphBest = new_states[i].state_graph
                for (n = 0; n < graphBest.GetNumOfNodes(); n++):
                    pn = new_states[i].state_room_positions[n]
                    graphBest.GetNode(n).SetPos(pn)


                layout_best = get_layout(&graphBest, new_states[i].state_room_positions)
                #ofstream fout
#ifdef DUMP_PARTIAL_SOLUTION
                graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'partial_%03d.xml', num_partials)).c_str())
                OpenDoors(layout_best, &graphBest, True)
                layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'partial_%03d.svg', num_partials)).c_str(), 800, 800, True, &graphBest)
#endif
                #m_bestSolCount ++
                num_partials += 1
                state_stack.insert(0, new_states[i])



    self.layout = layout_best
    self.layout.move_to_scene_centre()

#ifndef PERFORMANCE_TEST
        print(f'Total # of backtracks: {self.backtrack_count})
        #ofstream fout
        #fout.open(CLevelConfig.AddOutputPrefix('log.txt').c_str(), std.ios_base.app)
        #fout}Total # of backtracks: {self.backtrack_count << std.endl

#endif


def solve_1d_chain(self, indices, weighted_indices, old_state, new_states):
    if FLAG_USE_ILS:
        return self.solve_1d_chainILS(indices, old_state, new_states)

    graph = old_state.state_graph
    SetSequenceAs1Dchain(indices, graph)
    new_states.clear()
    if graph.GetNode(indices[0]).GetFlagFixed():
        old_state.insert_to_new_states(new_states, graph)
        return True

    flag_last_chain = graph.VisitedAllNodes()

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
    layout_best = get_layout(graph, old_state.state_room_positions)
    graphBest = *graph

    if not CLevelConfig.m_flagRandomWalk:
#if 0
        for (i = 0; i < weighted_indices.size(); i++):
            pickedRoom = layout_best.get_room((*weighted_indices)[i])
            SampleConfigSpaceForPickedRoom(layout_best, graph, indices, (*weighted_indices)[i])

#else # New on 09/24/2013: connect the rooms together as initial guess
        numOfVisitedNodes = int(indices.size() - weighted_indices.size())
        if (numOfVisitedNodes == 0): # The first chain...
            numOfVisitedNodes = 1

        std.vector<int> indicesVisited(numOfVisitedNodes)
        std.vector<bool> flagsVisited(graph.GetNumOfNodes(), False)
        for (i = 0; i < numOfVisitedNodes; i++):
            indicesVisited[i] = indices[i]
            flagsVisited[indices[i]] = True

        while (indicesVisited.size() < indices.size()):
            idxUnvisited = -1
            idxVisited = -1
            for (i = 0; i < int(weighted_indices.size()); i++):
                idx = (*weighted_indices)[i]
                if flagsVisited[idx]:
                    continue

                std.vector<int> connectedIndices = GetConnectedIndices(graph, idx, False)
                std.vector<int> visitedNeighbors
                for (j = 0; j < int(connectedIndices.size()); j++):
                    idxOther = connectedIndices[j]
                    if flagsVisited[idxOther]:
                        visitedNeighbors.push_back(idxOther)

                if not visitedNeighbors.empty():
                    idxUnvisited = idx
                    roomUnvisited = layout_best.get_room(idxUnvisited)
                    random_shuffle(visitedNeighbors.begin(), visitedNeighbors.end())
                    CConfigSpace configSpace(layout_best.get_room(visitedNeighbors[0]), roomUnvisited)
                    for (j = 1; j < int(visitedNeighbors.size()); j++):
                        CConfigSpace configSpaceTmp(layout_best.get_room(visitedNeighbors[j]), roomUnvisited)
                        configSpaceNew = CConfigSpace.FindIntersection(configSpace, configSpaceTmp)
                        if configSpaceNew.IsEmpty():
                            break
                        else:
                            configSpace = configSpaceNew


                    indicesVisited.push_back(idxUnvisited)
                    flagsVisited[idxUnvisited] = True
    #if 1 # Smartly sample the configuration space based on the energy...
                    std.vector<v2f> vecPos = configSpace.SmartlySampleConfigSpace()
                    idxBest = -1
                    energy_min = 1e10
                    graph_tmp = *graph
                    for (j = 0; j < graph_tmp.GetNumOfNodes(); j++):
                        graph_tmp.GetNode(j).SetFlagVisited(flagsVisited[j])

                    for (j = 0; j < int(vecPos.size()); j++):
                        layout_tmp = layout_best
                        pickedRoomTmp = layout_tmp.get_room(idxUnvisited)
                        dp = vecPos[j] - pickedRoomTmp.get_room_centre()
                        pickedRoomTmp.translate_room(dp)
                        collide_area = 0.f
                        connectivity = 0.f
                        energy_tmp = get_layout_energy(layout_tmp, &graph_tmp, collide_area, connectivity)
                        if collide_area < energy_min:
                            energy_min = collide_area
                            idxBest = j


                    if idxBest >= 0:
                        dp = vecPos[idxBest] - roomUnvisited.get_room_centre()
                        roomUnvisited.translate_room(dp)

    #else:
                    pos = configSpace.RandomlySampleConfigSpace()
                    dp = pos - roomUnvisited.get_room_centre()
                    roomUnvisited.translate_room(dp)
    #endif
                    break

#endif
    self.layout = layout_best
#ifdef DUMP_INTERMEDIATE_OUTPUT
    layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'chainInit_%03d.svg', self.chain_count)).c_str())
#endif
    collide_area = 0.0
    connectivity = 0.0
    energy_min = self.get_layout_energy(layout_best, graph, collide_area, connectivity, -1, True, indices)
    energy_current = energy_min
#ifdef DUMP_INTERMEDIATE_OUTPUT
    print(f'Initial energy: {energy_min}')
    #std.ofstream fout
    ##fout.open(CLevelConfig.AddOutputPrefix('log.txt').c_str(), std.ios_base.app)
    #fout << m_bestSolCount}\t{energy_min << std.endl
    for (n = 0; n < graph.GetNumOfNodes(); n++):
        pn = layout_best.get_room_positions()[n]
        graph.GetNode(n).SetPos(pn)

    graph.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.xml', m_bestSolCount)).c_str())
    OpenDoors(layout_best, graph, True)
    layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.svg', m_bestSolCount)).c_str())
    m_bestSolCount++
#endif
    pick_index_count = 0
    num_failures = 0
    for (i = 0; i < n; i++):
#ifndef PERFORMANCE_TEST
        print(f'Cycle {i + 1}/{n} (failures {num_failures}) ...\n')
#endif
        flag_was_accepted = False
        if num_failures > 10:
            if Random2(2) == 0:
#ifndef PERFORMANCE_TEST
                print(f'RANDOM RESTART CALLED! 11+ failures')
#endif
                break


        elif num_failures > 8:
            if Random2(3) == 0:
#ifndef PERFORMANCE_TEST
                print(f'RANDOM RESTART CALLED!  9+ failures')
#endif
                break


        elif num_failures > 5:
            if Random2(4) == 0:
#ifndef PERFORMANCE_TEST
                print(f'RANDOM RESTART CALLED!  6+ failures')
#endif
                break


        elif num_failures > 3:
            if Random2(6) == 0:
#ifndef PERFORMANCE_TEST
                print(f'RANDOM RESTART CALLED!  4+ failures')
#endif
                break


        elif num_failures > 0:
            if Random2(8) == 0:
#ifndef PERFORMANCE_TEST
                print(f'RANDOM RESTART CALLED: 1 failure but we just felt like quittin')
#endif
                break

        for (j = 0; j < m; j++):
            graph_tmp = graph
            layout_tmp = self.layout
            adjusted_index = RandomlyAdjustOneRoom(layout_tmp, &graph_tmp, indices, weighted_indices)
#if 0 # New on 08/16/2013
            if self.flag_visited_node:
                p_min = Vector2(1e10, 1e10)
                p_max = Vector2(-1e10, -1e10)
                for d in range(len(indices)):
                    idx = indices[d]
                    pj = layout_tmp.get_room(idx).get_room_centre()
                    for k in range(2):
                        p_min[k] = min(p_min[k], pj[k])
                        p_max[k] = max(p_max[k], pj[k])

                pos_cen = (p_min + p_max) * 0.5f
                for d in range(len(indices)):
                    idx = indices[d]
                    layout_tmp.get_room(idx).translate_room(-pos_cen)

#endif
            energy_tmp = get_layout_energy(layout_tmp, graph_tmp, collide_area, connectivity, adjusted_index, True, indices)

            # accept = get_layout_energyEarlyOut(layout_tmp, &graph_tmp, collide_area, connectivity, adjusted_index, &energy_tmp, energy_current )

            if ((CLevelConfig.m_flagRandomWalk and collide_area <= 1e-3 and connectivity <= 1e-3) or
                (collide_area <= 1e-4 and connectivity <= 1e-4)):
                new_state = old_state
                new_state.state_graph = graph_tmp
                new_state.state_room_positions = layout_tmp.get_room_positions()
                new_state.state_energy = energy_tmp
                new_state.move_rooms_to_scene_center(graph_tmp)
                new_state.insert_to_new_states(new_states, graph_tmp)
                if new_states.size() >= CLevelConfig.m_numOfSolutionsToTrack:
                    return True

                if flag_last_chain and (self.solution_count + int(new_states.size()) >= CLevelConfig.TARGET_NUM_SOLUTIONS):
                    return True

                #new_states.push_back(new_state)

            flag_accept = False

            if energy_tmp < energy_current:
                # Energy is lower, accept
                flag_accept = True
                if energy_tmp < energy_min:
                    energy_min = energy_tmp
#ifndef PERFORMANCE_TEST
                    print(f'A minimum energy: {energy_min}')
#endif
#ifdef DUMP_INTERMEDIATE_OUTPUT
                    layout_best = layout_tmp
                    graphBest = graph_tmp
                    for (n = 0; n < graphBest.GetNumOfNodes(); n++):
                        pn = layout_best.get_room_positions()[n]
                        graphBest.GetNode(n).SetPos(pn)

                    #std.ofstream fout
                    ##fout.open(CLevelConfig.AddOutputPrefix('log.txt').c_str(), std.ios_base.app)
                    #fout << m_bestSolCount}\t{energy_min << std.endl
                    graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.xml', m_bestSolCount)).c_str())
                    OpenDoors(layout_best, &graphBest, True)
                    layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.svg', m_bestSolCount)).c_str())
                    m_bestSolCount++
#endif
            else:
                DeltaE = std.abs(energy_tmp - energy_current)

                # Energy is higher...
                if i == 0 and j == 0:
                    DeltaE_avg = DeltaE
                    DeltaE_avg *= CLevelConfig.m_deltaEscaling

                # Generate probability of acceptance...
                prob = exp(-(energy_tmp - energy_current) / (DeltaE_avg * t))
                r = rand() / float(RAND_MAX)
                if r < prob:
                    flag_accept = True
                else:
                    flag_accept = False

            if flag_accept:
                DeltaE = std.abs(energy_tmp - energy_current)

                self.layout = layout_tmp
                graph = graph_tmp
                energy_current = energy_tmp
                if DeltaE != 0.0:
                    na += 1
                    DeltaE_avg = (DeltaE_avg * (na - 1.0f) + DeltaE) / float(na)
                flag_was_accepted = True

            pick_index_count +=1
            pick_index_count = pick_index_count % int(len(indices))

        if not flag_was_accepted:
            num_failures += 1

        # Lower the temperature for next cycle
        t *= frac

#ifndef PERFORMANCE_TEST
    print(f'Final energy: {energy_min}')
#endif
    if not new_states:
#ifdef DUMP_INTERMEDIATE_OUTPUT
        print(f'Empty solution set!')
        graphBest.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'backTracking_level%02d_%03d.xml', self.backtrack_level, self.backtrack_count)).c_str())
        for (n = 0; n < graphBest.GetNumOfNodes(); n++):
            graphBest.GetNode(n).SetFlagVisited(False)

        for (n = 0; n < int(indices.size()); n++):
            graphBest.GetNode(indices[n]).SetFlagVisited(True)

        for (n = 0; n < int(weighted_indices.size()); n++):
            graphBest.GetNode((*weighted_indices)[n]).SetFlagVisited(False)

        layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'backTracking_level%02d_%03d.svg', self.backtrack_level, self.backtrack_count)).c_str())
        layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'backTrackingPartial_level%02d_%03d.svg', self.backtrack_level, self.backtrack_count)).c_str(), 400, 400, True, &graphBest)
#endif
        return False

#ifndef PERFORMANCE_TEST
    print(f'Number of valid states: {len(new_states)})
#endif
    sort(new_states.begin(), new_states.end(), CompareStateEnergySmallerFirst)
    numOfSolutionsToTrack = min(int(new_states.size()), CLevelConfig.m_numOfSolutionsToTrack)
    std.vector<CurrentState> newer_states(numOfSolutionsToTrack)
    for (i = 0; i < numOfSolutionsToTrack; i++):
        newer_states[i] = new_states[i]

    new_states = newer_states

    return True


def self.solve_1d_chainILS(self, indices, old_state, new_states):
    graph = &(old_state.state_graph)
    SetSequenceAs1Dchain(indices, graph)
    new_states.clear()
    if graph.GetNode(indices[0]).GetFlagFixed() == True:
        old_state.insert_to_new_states(new_states, graph)
        return True


    # Borrow the parameters from simulated annealing...
     n = CLevelConfig.m_saNumOfCycles
     m = CLevelConfig.m_saNumOfTrials
    # CRoomLayout layout_best; # Current best result so far
    collide_area = 0.f
    connectivity = 0.f
    energy_min = 1e10
    energyHistory = 1e10
    pick_index_count = 0
    for (i = 0; i < n; i++):
        layoutHistory = self.layout
        graphHistory = *graph
        if i != 0:
            # Introduce perturbation...
            RandomlyAdjustOneRoom(self.layout, graph, indices, NULL)

        energy_tmp = get_layout_energy(self.layout, graph, collide_area, connectivity)
        energy_current = energy_tmp
        if i == 0:
            energy_min = energy_current
            print(f'Initial energy: {energy_current}')

        for (j = 0; j < m; j++):
            graph_tmp = *graph
            layout_tmp = self.layout
            RandomlyAdjustOneRoom(layout_tmp, &graph_tmp, indices, NULL)
#if 1 # New on 08/16/2013
            if self.flag_visited_node == True:
                v2f p_min(1e10)
                v2f p_max(-1e10)
                for (d = 0; d < int(indices.size()); d++):
                    idx = indices[d]
                    pj = layout_tmp.get_room(idx).get_room_centre()
                    for (k = 0; k < 2; k++):
                        p_min[k] = min(p_min[k], pj[k])
                        p_max[k] = max(p_max[k], pj[k])


                pos_cen = (p_min + p_max) * 0.5f
                for (d = 0; d < int(indices.size()); d++):
                    idx = indices[d]
                    layout_tmp.get_room(idx).translate_room(-pos_cen)


#endif
            energy_tmp = get_layout_energy(layout_tmp, &graph_tmp, collide_area, connectivity)
            if collide_area <= g_numericalTolerance and connectivity <= g_numericalTolerance:
                new_state = old_state
                new_state.state_graph = *graph
                new_state.state_room_positions = layout_tmp.get_room_positions()
                new_state.state_energy = energy_tmp
                new_state.move_rooms_to_scene_center(graph)
                new_state.insert_to_new_states(new_states, graph)

            if energy_tmp < energy_current:
                if energy_tmp < energy_min:
                    # layout_best = layout_tmp
                    energy_min = energy_tmp
#ifndef PERFORMANCE_TEST
                    print(f'A minimum energy: {energy_min}')
#endif

                self.layout = layout_tmp
                *graph = graph_tmp
                energy_current = energy_tmp

            pick_index_count++
            pick_index_count = pick_index_count % int(indices.size())

        if i == 0 or energy_min < energyHistory:
            energyHistory = energy_min

        else:
            self.layout = layoutHistory
            *graph = graphHistory


    print(f'Final energy: {energy_min}')
    if new_states.empty() == True:
        print(f'Empty solution setnot \n'
        return False

    print(f'Number of valid states: {len(new_states)}')
    sort(new_states.begin(), new_states.end(), CompareStateEnergySmallerFirst)
    numOfSolutionsToTrack = min(int(new_states.size()), CLevelConfig.m_numOfSolutionsToTrack)
    std.vector<CurrentState> newer_states
    for (i = 0; i < numOfSolutionsToTrack; i++):
        newer_states.push_back(new_states[i])

    new_states = newer_states

    return True


def set_current_state(self, s):
    self.graph = s.state_graph
    self.room_positions = s.state_room_positions
    self.layout = get_layout(self.graph, room_positions)


def SetSequenceAs1Dchain(self, indices, graph):
    m_sequence.clear()
    for (i = 0; i < int(indices.size()); i++):
        idx = graph.GetNode(indices[i]).GetType()
        idx = idx % m_ptrTemplates.GetNumOfTemplates()
        m_sequence.push_back(idx)



def SetVisitedNeighbors(self, indices):
    m_visitedNeighbors.clear()
    m_visitedNeighbors.resize(indices.size())
    for (i = 0; i < int(m_visitedNeighbors.size()); i++):
        nodeIdx = indices[i]
        std.vector<int>neighbors = self.graph.GetNode(nodeIdx).GetNeighbors()
        for (j = 0; j < int(neighbors.size()); j++):
            neighborIdx = neighbors[j]
            if self.graph.GetNode(neighborIdx).GetFlagVisited() == True:
                m_visitedNeighbors[i].push_back(neighborIdx)





def DumpSolutionIntoXML(self):
    graphSol = *self.graph
    for (i = 0; i < graphSol.GetNumOfNodes(); i++):
        pi = room_positions[i]
        graphSol.GetNode(i).SetPos(pi[0], pi[1])

    graphSol.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'dbg_%03d.xml', self.solution_count)).c_str())
    layoutSol = get_layout(self.graph, room_positions)
    OpenDoors(layoutSol, self.graph)
    layoutSol.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'dbg_%03d.svg', self.solution_count)).c_str())


def randomly_pick_one_room(self, layout):
    picked_room_index = random.randint(0, layout.num_rooms - 1)
    return picked_room_index


def randomly_pick_one_room(self, layout, indices, weighted_indices):
    if weighted_indices:
        std.vector<int> tmp_indices = *weighted_indices
        chainLength = int(tmp_indices.size())
        for (i = 0; i < indices.size(); i++):
            energy_tmp = layout.get_room(indices[i]).GetEnergy()
            if energy_tmp > 1.1:
                tmp_indices.push_back(indices[i])



        picked_room_index = int(rand() / float(RAND_MAX) * chainLength)
        picked_room_index = picked_room_index % chainLength
        picked_room_index = (tmp_indices)[picked_room_index]
        return picked_room_index

    else:
        chainLength = int(indices.size())
        picked_room_index = int(rand() / float(RAND_MAX) * chainLength)
        picked_room_index = picked_room_index % chainLength
        picked_room_index = indices[picked_room_index]
        return picked_room_index



def RandomlyPickAnotherRoom(self, layout, pickedIndex):
    numOfRooms = layout.GetNumOfRooms()
    otherRoomIndex = pickedIndex
    while (otherRoomIndex == pickedIndex)
        otherRoomIndex = int(rand() / float(RAND_MAX) * numOfRooms)
        otherRoomIndex = otherRoomIndex % numOfRooms
    return otherRoomIndex


def GetConnectedIndices(self, graph, pickedIndex, flagVisitedOnly ''' = True '''):
    std.vector<int> indices
    for (i = 0; i < graph.GetNumOfEdges(); i++):
        edge = graph.GetEdge(i)
        idx0 = edge.GetIdx0()
        idx1 = edge.GetIdx1()
        if idx0 != pickedIndex and idx1 != pickedIndex:
            continue

        idx = (idx0 == pickedIndex) ? idx1 : idx0
        if graph.GetNode(idx).GetFlagVisited() == False and flagVisitedOnly == True:
            continue

        indices.push_back(idx)

    return indices


def RandomlyAdjustOneRoom(self, layout, graph, indices, weighted_indices):
    numOfRooms = layout.GetNumOfRooms()
    if numOfRooms <= 1:
        return -1


    r = rand() / float(RAND_MAX)

#if 0 # Before 07/16/2013
    if  r < 0.25f :
        RandomlyAdjustOneRoom01(layout, graph, indices)

    elif  r < 0.5f :
        RandomlyAdjustOneRoom02(layout, graph, indices)

    elif  r < 0.75f or CLevelConfig.m_flagEnableTypeChange == False :
#else:
    if (r < 0.75f or CLevelConfig.m_flagEnableTypeChange == False) # nv: was 0.9
#endif
        if CLevelConfig.m_flagRandomWalk == False:
            return RandomlyAdjustOneRoom03(layout, graph, indices, weighted_indices)

        else:
            return GradientDescentOneRoom(layout, graph, *weighted_indices)


    else:
        return RandomlyAdjustOneRoom04(layout, graph, indices, weighted_indices)



def RandomlyAdjustOneRoom01(self, layout, graph, indices):
    picked_room_index = randomly_pick_one_room(layout, indices, NULL)
    pickedRoom = layout.get_room(picked_room_index)

    for (i = 0; i < pickedRoom.GetNumOfEdges(); i++):
        edge = pickedRoom.GetEdge(i)
        pr2 = edge.GetPos2() - edge.GetPos1()
        pr = v3f(pr2[0], pr2[1], 0.f)
        norm = v2f(pr[1], -pr[0])
        norm = normalize(norm)
        distMin = 1e10
        CRoomEdge edgeNearest
        for (j = 0; j < layout.GetNumOfRooms(); j++):
            if j == picked_room_index:
                continue

            if (graph.GetNode(j).GetFlagVisited() == False) continue
            otherRoom = layout.get_room(j)
            for (k = 0; k < otherRoom.GetNumOfEdges(); k++):
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
            pickedRoom.translate_room(dp)




def RandomlyAdjustOneRoom02(self, layout, graph, indices):
    picked_room_index = randomly_pick_one_room(layout, indices, NULL)
    pickedRoom = layout.get_room(picked_room_index)

    numOfEdges = pickedRoom.GetNumOfEdges()
    pickedEdgeIndex = int(rand() / float(RAND_MAX) * numOfEdges)
    pickedEdgeIndex = pickedEdgeIndex % numOfEdges
    edge = pickedRoom.GetEdge(pickedEdgeIndex)
    pr2 = edge.GetPos2() - edge.GetPos1()
    pr = v3f(pr2[0], pr2[1], 0.f)
    norm = v2f(pr[1], -pr[0])
    norm = normalize(norm)

    otherRoomIndex = RandomlyPickAnotherRoom(layout, picked_room_index)
    otherRoom = layout.get_room(otherRoomIndex)
    distMin = 1e10
    CRoomEdge edgeNearest

    for (k = 0; k < otherRoom.GetNumOfEdges(); k++):
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
        pickedRoom.translate_room(dp)



def GradientDescentOneRoom(self, layout, graph, indices):
    collide_area = 0.f
    connectivity = 0.f
    myEnergy = get_layout_energy(layout, graph, collide_area, connectivity)
    picked_room_index = randomly_pick_one_room(layout, indices, NULL)

    besti = -1
    bestEnergy = 10e6
    std.vector<int> candidateAngles

    for (i = 0; i < 360; i += 10):
        angle = i * atan(1.f) * 4.f / 180.0f
        l = layout
        pickedRoom = l.get_room(picked_room_index)

         one_step_length = 0.1f
        dp = v2f(cosf(angle) * one_step_length, sinf(angle) * one_step_length)

        pickedRoom.translate_room(dp)
        collide_area = 0.f
        connectivity = 0.f
        energy_min = get_layout_energy(l, graph, collide_area, connectivity)
        if energy_min < bestEnergy or besti == -1:
            bestEnergy = energy_min
            besti = i

        if energy_min < myEnergy:
            candidateAngles.push_back(i)



    #   angle = besti * M_PI / 180.0f
    float angle

    if candidateAngles.empty():
        # Wellnot  I'm stuck on a local minimum.
        angle = Random2(36) * 10 * atan(1.f) * 4.f / 180.0f

    else:
        angle = candidateAngles[Random2(candidateAngles.size())] * atan(1.f) * 4.f / 180.0f


    stepSize = 0.1f
    bestStep = -1
    bestEnergy = 10e6

    for (iters = 0; iters < 4; iters++):
        for (stepLength = stepSize; stepLength <= 2; stepLength += stepSize):
            l = layout
            pickedRoom = l.get_room(picked_room_index)

            dp = v2f(cosf(angle) * stepLength, sinf(angle) * stepLength)

            pickedRoom.translate_room(dp)
            collide_area = 0.f
            connectivity = 0.f
            energy_min = get_layout_energy(l, graph, collide_area, connectivity)
            if energy_min < bestEnergy or bestStep == -1:
                bestEnergy = energy_min
                bestStep = stepLength


        if myEnergy - bestEnergy < 0.00001:
            stepSize /= 2.0f



    dp = v2f(cosf(angle) * bestStep, sinf(angle) * bestStep)

    # do the actual translation on the room itself
    pickedRoom = layout.get_room(picked_room_index)
    pickedRoom.translate_room(dp)
    return picked_room_index


def RandomlyAdjustOneRoom03(self, layout, graph, indices, weighted_indices):
    picked_room_index = randomly_pick_one_room(layout, indices, weighted_indices)
    pickedRoom = layout.get_room(picked_room_index)

    SampleConfigSpaceForPickedRoom(layout, graph, indices, picked_room_index)
    return picked_room_index


def SampleConfigSpaceForPickedRoom(self, layout, graph, indices, picked_room_index):
    pickedRoom = layout.get_room(picked_room_index)
    CConfigSpace configSpace
    std.vector<int> connectedIndices = GetConnectedIndices(graph, picked_room_index)
    if connectedIndices.size() >= 1:
        random_shuffle(connectedIndices.begin(), connectedIndices.end())
        idx0 = connectedIndices[0]
        CConfigSpace configSpace0(layout.get_room(idx0), pickedRoom)
        configSpace = configSpace0
        for (i = 1; i < int(connectedIndices.size()); i++):
            CConfigSpace configSpaceTmp(layout.get_room(connectedIndices[i]), pickedRoom)
            configSpaceNew = CConfigSpace.FindIntersection(configSpace, configSpaceTmp)
            if configSpaceNew.IsEmpty() == True:
                break

            else:
                configSpace = configSpaceNew



    whileCnt = 0
    while (configSpace.IsEmpty() == True)
        otherRoomIndex = RandomlyPickAnotherRoom(layout, picked_room_index)
        otherRoom = layout.get_room(otherRoomIndex)
        configSpace = CConfigSpace(otherRoom, pickedRoom)
        whileCnt++
        if whileCnt >= 1000:
            print(f'Break from the while loop after reaching enough number of trials!')
            return


    pos = configSpace.RandomlySampleConfigSpace()
    dp = pos - pickedRoom.get_room_centre()
    pickedRoom.translate_room(dp)


def RandomlyAdjustOneRoom04(self, layout, graph, indices, weighted_indices):
    numOfTemplates = m_ptrTemplates.GetNumOfTemplates()
    if numOfTemplates <= 1:
        return -1


    picked_room_index = randomly_pick_one_room(layout, indices, weighted_indices)
    pickedRoom = layout.get_room(picked_room_index)

    typeOld = graph.GetNode(picked_room_index).GetType()
    typeNew = typeOld
    boundaryOld = graph.GetNode(picked_room_index).GetBoundaryType()
    boundaryNew = -1
    whileCnt = 0
    while (typeNew == typeOld or boundaryNew != boundaryOld or m_ptrTemplates.get_room(typeNew).GetBoundaryType() == 1)
        typeNew = int(rand() / float(RAND_MAX) * numOfTemplates)
        typeNew = typeNew % numOfTemplates
        boundaryNew = m_ptrTemplates.get_room(typeNew).GetBoundaryType()
        whileCnt++
        if whileCnt >= 1000:
            print(f'Break from the while loop after reaching enough number of trials in RandomlyAdjustOneRoom04()!')
            return -1


    graph.GetNode(picked_room_index).SetType(typeNew)
    room = m_ptrTemplates.get_room(typeNew)
    p1 = room.get_room_centre()
    p2 = pickedRoom.get_room_centre()
    dp = p2 - p1
    room.translate_room(dp)
    pickedRoom = room
#if 1 # New on 09/15/2013
    #SampleConfigSpaceForPickedRoom(layout, graph, indices, picked_room_index)
#endif
    return picked_room_index


def get_layout_energyEarlyOut(self, layout, graph, collide_area, connectivity, roomMoved, energy_tmp, energy_current):
    layout.ResetRoomEnergies()
    *energy_tmp = 1.f

    # do connectivity first, it's (probably?) cheaper

    if CLevelConfig.m_sigmaConnectivity > 0.f:
        connectivity = CheckRoomConnectivity(layout, graph, True, roomMoved)
        (*energy_tmp) *= exp(connectivity * CLevelConfig.m_sigmaConnectivity)

    if *energy_tmp > energy_current:
        return False

    if CLevelConfig.m_sigmaCollide > 0.f:
        collide_area = LayoutCollide(layout, graph, True, roomMoved)
        (*energy_tmp) *= exp(collide_area * CLevelConfig.m_sigmaCollide)

    if *energy_tmp > energy_current:
        return False


    if CLevelConfig.m_sigmaContact > 0.f:
        contactArea = LayoutContact(layout, graph, True, CLevelConfig.m_flagNonOverlapContact)
        (*energy_tmp) *= exp(-contactArea * CLevelConfig.m_sigmaContact)

    return True


def get_layout_energy(self, layout, graph, collide_area, connectivity, roomMoved, doContact, indices):
    layout.ResetRoomEnergies()
    layoutEnergy = 1.f
    if CLevelConfig.m_sigmaCollide > 0.f:
        collide_area = LayoutCollide(layout, graph, True, roomMoved)
        layoutEnergy *= exp(collide_area * CLevelConfig.m_sigmaCollide)


    if CLevelConfig.m_sigmaConnectivity > 0.f:
        connectivity = CheckRoomConnectivity(layout, graph, True, roomMoved)
        layoutEnergy *= exp(connectivity * CLevelConfig.m_sigmaConnectivity)

    if CLevelConfig.m_sigmaContact > 0.f and doContact:
        contactArea = -LayoutContact(layout, graph, True, CLevelConfig.m_flagNonOverlapContact, indices)

        if contactArea >= 0.0f:
            contactArea = 0.0

        if contactArea < 0:
            layoutEnergy *= exp(contactArea / CLevelConfig.m_sigmaContact)



    return layoutEnergy


def CheckRoomConnectivity(self, layout, graph, flagVisitedOnly ''' = False ''', roomMoved):
    connectivity = 0.f
    if graph == NULL:
        return connectivity

    for (i = 0; i < graph.GetNumOfEdges(); i++):
        edge = graph.GetEdge(i)
        idx0 = edge.GetIdx0()
        idx1 = edge.GetIdx1()
        flagVisited0 = graph.GetNode(idx0).GetFlagVisited()
        flagVisited1 = graph.GetNode(idx1).GetFlagVisited()
        if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
            continue

        flagFixed0 = graph.GetNode(idx0).GetFlagFixed()
        flagFixed1 = graph.GetNode(idx1).GetFlagFixed()
        if flagFixed0 == True and flagFixed1 == True:
            continue

        if roomMoved == -1 or roomMoved == idx0 or roomMoved == idx1 or layout.cachedConnectivities.find(std.make_pair(idx0, idx1)) == layout.cachedConnectivities.end():
            contactArea = RoomContact(layout.get_room(idx0), layout.get_room(idx1))
            if contactArea <= CLevelConfig.m_roomContactThresh:
                if CLevelConfig.m_flagDiscreteConnectFunc == True:
                    connectivity += 1.f
                    layout.cachedConnectivities[std.make_pair(idx0, idx1)] = 1.f

                else:
                    d = RoomDistance(layout.get_room(idx0), layout.get_room(idx1))
                    d += CLevelConfig.m_roomContactThresh
                    layout.cachedConnectivities[std.make_pair(idx0, idx1)] = d
                    connectivity += d

                factor = 1.1f
                layout.get_room(idx0).UpdateEnergy(factor)
                layout.get_room(idx1).UpdateEnergy(factor)

            else:
                layout.cachedConnectivities[std.make_pair(idx0, idx1)] = 0.0f


        else:
            connectivity += layout.cachedConnectivities[std.make_pair(idx0, idx1)]



    return connectivity


def LayoutCollide(self, layout, graph, flagVisitedOnly ''' = False ''', roomThatMoved ''' = -1 '''):
    collide_areaTotal = 0.f
    collideCount = 0
    numOfRooms = layout.GetNumOfRooms()
    for (i = 0; i < numOfRooms; i++):
        for (j = i + 1; j < numOfRooms; j++):
            flagVisited0 = graph.GetNode(i).GetFlagVisited()
            flagVisited1 = graph.GetNode(j).GetFlagVisited()
            if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
                continue

            flagFixed0 = graph.GetNode(i).GetFlagFixed()
            flagFixed1 = graph.GetNode(j).GetFlagFixed()
            if flagFixed0 == True and flagFixed1 == True:
                continue

            if roomThatMoved == -1 or roomThatMoved == i or roomThatMoved == j or layout.cachedCollisionEnergies.find(std.make_pair(i, j)) == layout.cachedCollisionEnergies.end():
                collide_area = RoomCollides(layout.get_room(i), layout.get_room(j))
                if collide_area > 0.f:
                    collide_areaTotal += collide_area
                    collideCount++
                    factor = exp(collide_area)
                    layout.get_room(i).UpdateEnergy(factor)
                    layout.get_room(j).UpdateEnergy(factor)
                    layout.cachedCollisionEnergies[std.make_pair(i, j)] = collide_area

                else:
                    layout.cachedCollisionEnergies[std.make_pair(i, j)] = collide_area


            else:
                collide_areaTotal += layout.cachedCollisionEnergies[std.make_pair(i, j)]




#ifdef PRINT_OUT_DEBUG_INFO
    print(f'Number of colliding room pairs: {collideCount}')
    print(f'Total area of colliding area: {collide_areaTotal}')
#endif
    return collide_areaTotal


def LayoutCollide(self, layout):
    collide_areaTotal = 0.f
    collideCount = 0
    numOfRooms = layout.GetNumOfRooms()
    for (i = 0; i < numOfRooms; i++):
        for (j = i + 1; j < numOfRooms; j++):
            if layout.get_room(i).GetBoundaryType() == 1 and layout.get_room(j).GetBoundaryType() == 1:
                continue

            collide_area = RoomCollides(layout.get_room(i), layout.get_room(j))
            if collide_area > 0.f:
                collide_areaTotal += collide_area
                collideCount++



#ifdef PRINT_OUT_DEBUG_INFO
    print(f'Number of colliding room pairs: {collideCount}')
    print(f'Total area of colliding area: {collide_areaTotal}')
#endif
    return collide_areaTotal


def RoomCollides(self, room1, room2):
    collide_area = -1.f

    # Test the bounding box first...
    AABB2f bb1, bb2
    room1.get_roomBoundingBox(bb1)
    room2.get_roomBoundingBox(bb2)
    if TestBoundingBoxCollides(bb1, bb2) == False:
        return 0.f

    # Use the Clipper library...
    CClipperWrapper wrapper
    collide_area = wrapper.Computecollide_area(room1, room2)

    return collide_area


def BoundingBoxCollidesArea(self, bb1, bb2):
    collide_area = -1.f
    for (j = 0; j < 2; j++):
        if bb1.m_posMax[j] < bb2.m_posMin[j] or bb1.m_posMin[j] > bb2.m_posMax[j]:
            return collide_area


    collide_area = 1.f
    for (j = 0; j < 2; j++):
        p_min = max(bb1.m_posMin[j], bb2.m_posMin[j])
        p_max = min(bb1.m_posMax[j], bb2.m_posMax[j])
        if (p_min > p_max) return -1.f
        pd = p_max - p_min
        collide_area *= pd

    return collide_area


def TestBoundingBoxCollides(self, bb1, bb2):
    for (j = 0; j < 2; j++):
        if bb1.m_posMax[j] < bb2.m_posMin[j] or bb1.m_posMin[j] > bb2.m_posMax[j]:
            return False


    return True


def LayoutContact(self, layout, graph, flagVisitedOnly ''' = False ''', flagNonOverlap ''' = False ''', indices, roomThatMoved ''' probably == null '''):
    contactAreaTotal = 0.f
    contactCount = 0
    numOfRooms = layout.GetNumOfRooms()

    for (i = 0; i < numOfRooms; i++):
        flagVisited0 = graph.GetNode(i).GetFlagVisited()
        if flagVisited0 == False:
            continue


        badNeighbour = False
        std.vector<int>neighbours = graph.GetNode(i).GetNeighbors()
        for (j = 0; j < neighbours.size(); j++):
            found = False
            for (k = 0; k < indices.size(); k++):
                if (*indices)[k] == neighbours[j]:
                    found = True
                    break


            if not found:
                badNeighbour = True
                break


        if not badNeighbour:
            continue

        perimeter = RoomPerimeter(layout.get_room(i))

        for (j = i + 1; j < numOfRooms; j++):
            flagVisited1 = graph.GetNode(j).GetFlagVisited()
            if flagVisitedOnly == True and (flagVisited0 == False or flagVisited1 == False):
                continue

            if i == roomThatMoved or j == roomThatMoved or roomThatMoved == -1 or layout.cachedContacts.find(std.make_pair(i, j)) == layout.cachedContacts.end():
                if RoomCollides(layout.get_room(i), layout.get_room(j)) > 0.f:
                    layout.cachedContacts[std.make_pair(i, j)] = 0.0f
                    continue

                contactArea = RoomContact(layout.get_room(i), layout.get_room(j))
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
    print(f'Number of contacting room pairs: {contactCount}')
    print(f'Total area of contacting area: {contactAreaTotal}')
#endif
    return contactAreaTotal


def ComputeLabelPosition(self, idx, graph, labelRad):
    v2f p_min, p_max
    graph.GetGraphBoundingBox(p_min, p_max)
    vMax = max(max(std.abs(p_min[0]), std.abs(p_min[1])), max(std.abs(p_max[0]), std.abs(p_max[1])))
     pos = graph.GetNodePos(idx)
     rad = labelRad
    n = 32
    dMinMax = -1e10
    v2f piMax
    for (i = 0; i < n; i++):
        angle = atan(1.f) * 8.f * float(i) / float(n)
        cv = -cos(angle) * rad
        sv = sin(angle) * rad
        pi = pos + v2f(cv, sv)
        if pi[0] < -vMax or pi[0] > vMax or pi[1] < -vMax or pi[1] > vMax:
            continue

        dMin = 1e10
        for (j = 0; j < graph.GetNumOfEdges(); j++):
            edge = graph.GetEdge(j)
            idx1 = edge.GetIdx0()
            idx2 = edge.GetIdx1()
            if idx1 != idx and idx2 != idx:
                continue

            pos1 = graph.GetNodePos(idx1)
            pos2 = graph.GetNodePos(idx2)
            dTmp = PointToSegmentSqDistance(pi, CLineBase(pos1, pos2))
            if dTmp < dMin:
                dMin = dTmp


        if dMin >= rad * rad:
            return pi

        if dMin > dMinMax:
            dMinMax = dMin
            piMax = pi


    return piMax

