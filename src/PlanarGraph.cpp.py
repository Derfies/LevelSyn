#include "PlanarGraph.h"

CGraphFace CPlanarGraph.m_faceTmp
std.vector<CGraphFace> CPlanarGraph.m_faces

CPlanarGraph.CPlanarGraph()
    m_pickedNodeIndex = -1
    m_numOfTypes = 1


def ClearGraph(self):
    m_nodes.clear()
    m_edges.clear()


def PrintGraph(self):
    std.cout << "The graph contains " << m_nodes.size() << " nodes and " << m_edges.size() << " edgesnot \n"
    std.cout << "Visited flags: "
    for (i = 0; i < int(m_nodes.size()); i++)
        std.cout << m_nodes[i].GetFlagVisited() << " "

    std.cout << std.endl
    std.cout << "Node types: "
    for (i = 0; i < int(m_nodes.size()); i++)
        std.cout << m_nodes[i].GetType() << " "

    std.cout << std.endl


def LoadGraphFromXML(self, fileName, flagDetectFaces ''' = True ''', flagIgnoreIndiv ''' = True '''):
    # Clear the current graph...
    ClearGraph()

    tinyxml2.XMLDocument doc
    if doc.LoadFile(fileName) != tinyxml2.XML_SUCCESS:
        std.cout << "Failed to load graph from " << fileName << "not \n"
        return False

    #doc.Print()

    xmlRoot = doc.RootElement()
    assert(xmlRoot)

    xmlNode = xmlRoot.FirstChild()
    while (xmlNode != 0)
        if strcmp(xmlNode.Value(), "Node") == 0:
            # Parse a node...
            CGraphNode graphNode
            float px, py
            rx = xmlNode.ToElement().QueryFloatAttribute("px", &px)
            ry = xmlNode.ToElement().QueryFloatAttribute("py", &py)
            if rx != tinyxml2.XML_SUCCESS or ry != tinyxml2.XML_SUCCESS:
                graphNode.RandomlyInitPos()

            else:
                graphNode.SetPos(px, py)

            int type
            r = xmlNode.ToElement().QueryIntAttribute("type", &type)
            if r == tinyxml2.XML_SUCCESS:
                graphNode.SetType(type)

            int boundary
            rb = xmlNode.ToElement().QueryIntAttribute("boundary", &boundary)
            if rb == tinyxml2.XML_SUCCESS:
                graphNode.SetBoundaryType(boundary)

            int fixed
            rf = xmlNode.ToElement().QueryIntAttribute("fix", &fixed)
            if rf == tinyxml2.XML_SUCCESS and fixed != 0:
                graphNode.SetFlagFixed(True)

             str = xmlNode.ToElement().Attribute("name")
            if *str != '\0':
                graphNode.SetName(str)

            AddGraphNode(graphNode)

        elif strcmp(xmlNode.Value(), "Edge") == 0:
            # Parse an edge...
            idx0 = -1
            idx1 = -1
            r0 = xmlNode.ToElement().QueryIntAttribute("node0", &idx0)
            r1 = xmlNode.ToElement().QueryIntAttribute("node1", &idx1)
            if r0 != tinyxml2.XML_SUCCESS:
                 str = xmlNode.ToElement().Attribute("name0")
                idx0 = FindNodeAccordingToName(str)

            if r1 != tinyxml2.XML_SUCCESS:
                 str = xmlNode.ToElement().Attribute("name1")
                idx1 = FindNodeAccordingToName(str)

            if idx0 >= 0 and idx1 >= 0:
                CGraphEdge graphEdge(idx0, idx1)
                AddGraphEdge(graphEdge)



        # Move to the next sibling...
        xmlNode = xmlNode.NextSibling()


    SetNodeNeighbors()
    if flagIgnoreIndiv == True:
        RemoveIndividualNodes()

    if flagDetectFaces == False:
        return False

    #PrintGraph()

    # Step 1: Detect faces of the planar graph...
    DetectFaces()
    return True


def SaveGraphAsXML(self, fileName):
     str = "\t<?xml version=\"1.0\" standalone=\"yes\" ?>\n"
                      "<not -- graph input data -.\n"
                      "<Graph>\n"
                      "</Graph>\n"
    tinyxml2.XMLDocument doc
    doc.Parse(str)
    root = doc.RootElement()
    # Dump nodes...
    for (i = 0; i < int(m_nodes.size()); i++)
        nodeElem = doc.NewElement("Node")
        nodeElem.SetAttribute("name", m_nodes[i].GetName().c_str())
        std.ostringstream oss0
        std.ostringstream oss1
        oss0 << m_nodes[i].GetPos()[0]
        oss1 << m_nodes[i].GetPos()[1]
        nodeElem.SetAttribute("px", oss0.str().c_str())
        nodeElem.SetAttribute("py", oss1.str().c_str())
        std.ostringstream ossType
        ossType << m_nodes[i].GetType()
        nodeElem.SetAttribute("type", ossType.str().c_str())
        std.ostringstream ossFixed
        ossFixed << m_nodes[i].GetFlagFixed()
        nodeElem.SetAttribute("fix", ossFixed.str().c_str())
        if m_nodes[i].GetBoundaryType() != 0:
            nodeElem.SetAttribute("boundary", m_nodes[i].GetBoundaryType())

        root.InsertEndChild(nodeElem)

    # Dump edges...
    for (i = 0; i < int(m_edges.size()); i++)
        edgeElem = doc.NewElement("Edge")
        edgeElem.SetAttribute("node0", m_edges[i].GetIdx0())
        edgeElem.SetAttribute("node1", m_edges[i].GetIdx1())
        root.InsertEndChild(edgeElem)


    saveFlag = doc.SaveFile(fileName)
    return saveFlag


def AddGraphEdge(self, edge):
    if CheckDuplicatedEdge(edge) == True:
        return False

    m_edges.push_back(edge)
    return True


def CheckDuplicatedEdge(self, edge):
    idx10 = edge.GetIdx0()
    idx11 = edge.GetIdx1()
    for (i = 0; i < GetNumOfEdges(); i++)
        edge0 = GetEdge(i)
        idx00 = edge0.GetIdx0()
        idx01 = edge0.GetIdx1()
        if (idx00 == idx10 and idx01 == idx11) or (idx00 == idx11 and idx01 == idx10):
            return True


    return False


def AddGraphNodes(self, numOfNodes, parent ''' = -1 '''):
    current = parent
    for (i = 0; i < numOfNodes; i++)
        CGraphNode node
        std.ostringstream os
        os << GetNumOfNodes()
        name = "node" + os.str()
        node.SetName(name)
        AddGraphNode(node)
        curId = GetNumOfNodes() - 1
        if current >= 0:
            AddGraphEdge(CGraphEdge(current, curId))

        current = curId



def SetNodeNeighbors(self):
    for (i = 0; i < int(m_nodes.size()); i++)
        m_nodes[i].ClearNeighbors()

    for (i = 0; i < int(m_edges.size()); i++)
        idx0 = m_edges[i].GetIdx0()
        idx1 = m_edges[i].GetIdx1()
        m_nodes[idx0].AddNeighbor(idx1)
        m_nodes[idx1].AddNeighbor(idx0)



def PickClosestNode(self, px, py):
    distSqrMin = std.numeric_limits<float>.max()
    v2f p
    p[0] = px
    p[1] = py
    for (i = 0; i < int(m_nodes.size()); i++)
        pi = m_nodes[i].GetPos()
        distSqrTmp = mag2(pi - p)
        if distSqrTmp < distSqrMin:
            distSqrMin = distSqrTmp
            m_pickedNodeIndex = i


    std.cout << "Have picked the " << m_pickedNodeIndex << "th node centered at " << GetNodePos(m_pickedNodeIndex) << "not \n"


def MovePickedNode(self, dx, dy):
    if m_pickedNodeIndex < 0:
        return

    pos = m_nodes[m_pickedNodeIndex].GetPos()
    pos[0] += dx
    pos[1] += dy
    m_nodes[m_pickedNodeIndex].SetPos(pos)


def RandomInitGraph(self):
    ClearGraph()

    minSkeletonSize = 5
    additionalSkeletonSize = Random2(5)
    mainSkeletonSize = minSkeletonSize + additionalSkeletonSize
    AddGraphNodes(mainSkeletonSize)

    # Add blue key branch: don't let self happen in the main room
    blueLoop = Random2(minSkeletonSize - 1) + 1

    # add red key branch: don't let self happen in the main room
    redLoop = Random2(minSkeletonSize - 1) + 1

    blueSectionLength = 4 + Random2(4)
    redSectionLength = 4 + Random2(4)

    if blueLoop < mainSkeletonSize:
        AddGraphNodes(blueSectionLength, blueLoop)
        if CoinFlip():
            idx0 = GetNumOfNodes() - 1
            idx1 = Random2(minSkeletonSize)
            AddGraphEdge(CGraphEdge(idx0, idx1))



    if redLoop < mainSkeletonSize:
        AddGraphNodes(redSectionLength, redLoop)
        if CoinFlip():
            idx0 = GetNumOfNodes() - 1
            idx1 = Random2(minSkeletonSize)
            AddGraphEdge(CGraphEdge(idx0, idx1))



    # Add decoration
    numStubs = Random2(5)
    for (i = 0; i < numStubs; i++)
        offset = Random2(mainSkeletonSize)
        AddGraphNodes(1 + Random2(2), offset)



def RandomInitPositions(self):
    for (i = 0; i < GetNumOfNodes(); i++)
        GetNode(i).RandomlyInitPos()



def RandomInitTypes(self):
    for (i = 0; i < GetNumOfNodes(); i++)
        type = int(rand() / float(RAND_MAX) * m_numOfTypes)
        GetNode(i).SetType(type)



def DetectFaces(self):
    # Based on the example in (http:#www.boost.org/doc/libs/1_47_0/libs/graph/example/planar_face_traversal.cpp)
    numOfNodes = GetNumOfNodes()
    Graph g(numOfNodes)

    numOfEdges = GetNumOfEdges()
    for (i = 0; i < numOfEdges; i++)
        idx0 = GetEdge(i).GetIdx0()
        idx1 = GetEdge(i).GetIdx1()
        add_edge(idx0, idx1, g)


    # Initialize the interior edge index
    property_map<Graph, e_index = get(edge_index, g)
    graph_traits<Graph>edge_count = 0
    graph_traits<Graph>.edge_iterator ei, ei_end
    for (boost.tuples.tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        put(e_index, *ei, edge_count++)


    typedef std.vector<graph_traits<Graph>.edge_descriptor> vec_t
    std.vector<vec_t> embedding(num_vertices(g))
#if 0
	# Test for planarity - we know it is planar, just want to
	# compute the planar embedding as a side-effect
	if ( boyer_myrvold_planarity_test(boyer_myrvold_params.graph = g,
		boyer_myrvold_params.embedding = &embedding[0]	) )
		std.cout << "Input graph is planar" << std.endl

	else:
		std.cout << "Input graph is not planar" << std.endl

#else:
    # Compute the planar embedding based on node positions...
    VertexIterator vi, vi_end
    for (boost.tie(vi, vi_end) = boost.vertices(g); vi != vi_end; ++vi)
        OutEdgeIterator ei, ei_end
        std.vector<EdgeDescriptor> adjacentEdges
        for (boost.tie(ei, ei_end) = boost.out_edges(*vi, g); ei != ei_end; ++ei)
            v1 = boost.source(*ei, g)
            v2 = boost.target(*ei, g)
            adjacentEdges.push_back(*ei)

        SortAdjacentVertices(g, *vi, adjacentEdges)
        for (i = 0; i < adjacentEdges.size(); ++i)
            std.cout << *vi << " . " << adjacentEdges[i] << std.endl

        if adjacentEdges.size() > 0:
            std.cout << std.endl
        embedding[*vi] = adjacentEdges

#endif

    std.cout << std.endl
              << "Vertices on the faces: " << std.endl
    vertex_output_visitor v_vis
    planar_face_traversal(g, &embedding[0], v_vis)

    std.cout << std.endl
              << "Edges on the faces: " << std.endl
    edge_output_visitor e_vis
    planar_face_traversal(g, &embedding[0], e_vis)

    RemoveTheOutsideFace()


def SortAdjacentVertices(self, g, vert, adjacentEdges):
    if adjacentEdges.size() < 1:
        return


    v1_0 = boost.source(adjacentEdges[0], g)
    v2_0 = boost.target(adjacentEdges[0], g)
    v_0 = (v1_0 == vert) ? v2_0 : v1_0
    edgeRef = GetNode(v_0).GetPos() - GetNode(vert).GetPos()
    for (i = 0; i < adjacentEdges.size(); ++i)
        for (j = i + 1; j < adjacentEdges.size(); ++j)
            v1_i = boost.source(adjacentEdges[i], g)
            v2_i = boost.target(adjacentEdges[i], g)
            v_i = (v1_i == vert) ? v2_i : v1_i
            v1_j = boost.source(adjacentEdges[j], g)
            v2_j = boost.target(adjacentEdges[j], g)
            v_j = (v1_j == vert) ? v2_j : v1_j
            edgePr1 = GetNode(v_i).GetPos() - GetNode(vert).GetPos()
            edgePr2 = GetNode(v_j).GetPos() - GetNode(vert).GetPos()
            if CompareEdgeDirections(edgePr1, edgePr2, edgeRef) == True:
                std.swap(adjacentEdges[i], adjacentEdges[j])





def CompareEdgeDirections(self, edgePr1, edgePr2, edgeRef):
#if 0 # Old and wrong version without the reference
	pr1 = v3f(edgePr1[0], edgePr1[1], 0.f)
	pr2 = v3f(edgePr2[0], edgePr2[1], 0.f)
	crossProduct = cross(pr1, pr2)
	bool flag =(crossProduct[2] < 0.f) ? True : False
	return flag
#else:
    pr1 = v3f(edgePr1[0], edgePr1[1], 0.f)
    pr2 = v3f(edgePr2[0], edgePr2[1], 0.f)
    pr0 = v3f(edgeRef[0], edgeRef[1], 0.f)
    cp1 = cross(pr0, pr1)
    cp2 = cross(pr0, pr2)
    # nothing is less than zero degrees
    if cp2[2] == 0.f and pr2[0] * pr0[0] + pr2[1] * pr0[1] >= 0.f:
        return False

    # zero degrees is less than anything else:
    if cp1[2] == 0.f and pr1[0] * pr0[0] + pr1[1] * pr0[1] >= 0.f:
        return True

    if cp1[2] * cp2[2] >= 0.f:
        # both on same side of reference, to each other
        cp = cross(pr1, pr2)
        return cp[2] > 0.f

    # vectors "less than" zero degrees are actually large, 2 pi
    return cp1[2] > 0.f
#endif


def RemoveTheOutsideFace(self):
    if m_faces.size() < 1:
        return

    maxFaceIdx = -1
    maxFaceSize = -1
    for (i = 0; i < int(m_faces.size()); i++)
        faceSizeTmp = m_faces[i].GetFaceSize()
        if faceSizeTmp > maxFaceSize:
            maxFaceSize = faceSizeTmp
            maxFaceIdx = i


    std.cout << "To remove the " << maxFaceIdx << "th face of size " << m_faces[maxFaceIdx].GetFaceSize() << "...\n"
    std.vector<CGraphFace> facesNew
    for (i = 0; i < int(m_faces.size()); i++)
        if i == maxFaceIdx:
            continue

        facesNew.push_back(m_faces[i])

    m_faces = facesNew

    for (i = 0; i < int(m_faces.size()); i++)
        std.cout << "The " << i << " th face: "
        for (j = 0; j < m_faces[i].GetFaceSize(); j++)
            std.cout << m_faces[i].GetIndices()[j] << " "

        std.cout << std.endl



def VisitedAllNodes(self):
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagVisited() == False:
            return False


    return True


def VisitedNoNode(self):
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagVisited() == True:
            return False


    return True


def HasFixedNode(self):
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagFixed() == True:
            return True


    return False


def GetFixedNodes(self):
    std.vector<int> indices
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagFixed() == True:
            indices.push_back(i)


    return indices


def GetUnfixedNodes(self):
    std.vector<int> indices
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagFixed() == False:
            indices.push_back(i)


    return indices


def ExtractDeepestFaceOrChain(self, flagCyclic, flagSmallFaceFirst):
    if VisitedNoNode() == True and HasFixedNode() == True:
        # Handle the fixed nodes first...
        std.vector<int> indices = GetFixedNodes()
        return indices

    if m_chains.empty() == False:
        for (i = 0; i < int(m_chains.size()); i++)
            chain = GetChain(i)
#if 0 # Before 09/23/2013, is NOT suitable for ghost cases
			idx0 = chain.GetIndices()[0]
			if  GetNode(idx0).GetFlagVisited() == True :
				continue

#else:
            flagAllVisited = True
            for (j = 0; j < chain.GetChainSize(); j++)
                if GetNode(chain.GetIndices()[j]).GetFlagVisited() == False:
                    flagAllVisited = False
                    break


            if flagAllVisited == True:
                continue

#endif
            flagCyclic = chain.GetFlagCyclic()
            std.vector<int> indices = chain.GetIndices()
            return indices


    std.vector<int> faceIndices = ExtractDeepestFace(flagSmallFaceFirst)
    if faceIndices.empty() == False and VisitedNoNode() == True:
        flagCyclic = True
        return faceIndices

    faceConstraintCount = CountConstraints(faceIndices)
    std.vector<int> chainIndices = ExtractDeepestChainNew()
    chainConstraintCount = CountConstraints(chainIndices)
    if faceIndices.empty() == False and faceConstraintCount > chainConstraintCount:
        flagCyclic = True
        return faceIndices

    else:
        flagCyclic = False
        return chainIndices



def ExtractDeepestFaceOrChainOld(self, flagCyclic, flagSmallFaceFirst):
    std.vector<int> faceIndices = ExtractDeepestFace(flagSmallFaceFirst)
    if faceIndices.empty() == False:
        flagCyclic = True
        return faceIndices

    flagCyclic = False
    std.vector<int> chainIndices = ExtractDeepestChain()
    return chainIndices


def ExtractDeepestFace(self, flagSmallFaceFirst):
    std.vector<int> faceIndices
    deepestFaceIdx = -1
    deepestFaceSize = 0
    smallestFaceIdx = -1
    smallestFaceSize = std.numeric_limits<int>.max()
    for (i = 0; i < GetNumOfFaces(); i++)
        face = GetFace(i)
        std.vector<int>indices = face.GetIndices()
        flagAllNodesUnvisited = True
        for (j = 0; j < face.GetFaceSize(); j++)
            if GetNode(indices[j]).GetFlagVisited() == True:
                flagAllNodesUnvisited = False
                break


        if flagAllNodesUnvisited == False:
            continue

        if face.GetFaceSize() > deepestFaceSize:
            deepestFaceIdx = i
            deepestFaceSize = face.GetFaceSize()

        if face.GetFaceSize() < smallestFaceSize:
            smallestFaceIdx = i
            smallestFaceSize = face.GetFaceSize()


    if deepestFaceIdx >= 0:
        faceIndices = GetFace(deepestFaceIdx).GetIndices()

    if flagSmallFaceFirst == True and smallestFaceIdx >= 0:
        faceIndices = GetFace(smallestFaceIdx).GetIndices()

    return faceIndices


def ExtractDeepestChainNew(self):
    chainLengthMin = std.numeric_limits<int>.max()
    chainLengthMax = 0
    chainLengthMinC = 0
    chainLengthMaxC = 0
    std.vector<int> chainMin
    std.vector<int> chainMax
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagVisited() == True:
            continue

        std.vector<int> chainIndices
        idx = i
        chainIndices.push_back(idx)
        GetNode(idx).SetFlagVisited(True)
        # Grow the chain with unvisited nodes...
        flagInserted = True
        while (flagInserted)
            flagInserted = False
            std.vector<int>neighbors = GetNode(idx).GetNeighbors()
            for (j = 0; j < int(neighbors.size()); j++)
                idxTmp = neighbors[j]
                if GetNode(idxTmp).GetFlagVisited() == False:
                    idx = idxTmp
                    chainIndices.push_back(idx)
                    GetNode(idx).SetFlagVisited(True)
                    flagInserted = True
                    break



        chainLengthTmp = int(chainIndices.size())
        # Set the visited flags back to False...
        for (j = 0; j < chainLengthTmp; j++)
            GetNode(chainIndices[j]).SetFlagVisited(False)

        chainConstraintCount = CountConstraints(chainIndices)
        if chainConstraintCount >= chainLengthMinC and chainLengthTmp < chainLengthMin:
            chainLengthMinC = chainConstraintCount
            chainLengthMin = chainLengthTmp
            chainMin = chainIndices

        if chainConstraintCount >= chainLengthMaxC and chainLengthTmp > chainLengthMax:
            chainLengthMaxC = chainConstraintCount
            chainLengthMax = chainLengthTmp
            chainMax = chainIndices


    return chainMin


def ExtractDeepestChain(self):
    std.vector<int> chainIndices
    # Almost randomly pick an unvisited node as the start of the chain...
    idx = -1
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetFlagVisited() == False:
            idx = i
            chainIndices.push_back(idx)
            GetNode(idx).SetFlagVisited(True)
            break


    # Grow the chain with unvisited nodes...
    flagInserted = True
    while (flagInserted)
        flagInserted = False
        std.vector<int>neighbors = GetNode(idx).GetNeighbors()
        for (i = 0; i < int(neighbors.size()); i++)
            idxTmp = neighbors[i]
            if GetNode(idxTmp).GetFlagVisited() == False:
                idx = idxTmp
                chainIndices.push_back(idx)
                GetNode(idx).SetFlagVisited(True)
                flagInserted = True
                break



    # Set the visited flags back to False...
    for (i = 0; i < int(chainIndices.size()); i++)
        GetNode(chainIndices[i]).SetFlagVisited(False)

    return chainIndices


def CountConstraints(self, indices):
    count = 0
    for i in range(len(indices)):
        idx = indices[i]
        neighbors = self.get_node(idx).get_neighbours()
        for j in range(len(neighbors)):
            idx_tmp = neighbors[j]
            if self.get_node(idx_tmp).get_flag_visited:
                count +=1
                break
    return count


def GetGraphBoundingBox(self, posMin, posMax):
    v2f pMin(1e10)
    v2f pMax(-1e10)
    for (i = 0; i < GetNumOfNodes(); i++)
        pi = GetNode(i).GetPos()
        for (j = 0; j < 2; j++)
            pMin[j] = min(pMin[j], pi[j])
            pMax[j] = max(pMax[j], pi[j])


    posMin = pMin
    posMax = pMax


def MoveGraphToSceneCenter(self):
    v2f posMin, posMax
    GetGraphBoundingBox(posMin, posMax)
    posCen = (posMin + posMax) * 0.5f
    for (i = 0; i < GetNumOfNodes(); i++)
        pi = GetNode(i).GetPos()
        pi = pi - posCen
        GetNode(i).SetPos(pi)



def ScaleGraphNodePositions(self, scaling):
    if scaling <= 0.f:
        return

    for (i = 0; i < GetNumOfNodes(); i++)
        pi = GetNode(i).GetPos()
        pi = pi * scaling
        GetNode(i).SetPos(pi)



def LoadChainsFromTXT(self, fileName):
    m_chains.clear()
    std.ifstream fin(fileName)
    if fin.fail() == True:
        std.cout << "Failed to load graph chains from file " << fileName << "not \n"
        return False


    int numOfChains
    fin >> numOfChains
    for (i = 0; i < numOfChains; i++)
        CGraphChain chain
        bool flag
        fin >> flag
        chain.SetFlagCyclic(flag)
        int numOfIndices
        fin >> numOfIndices
        for (j = 0; j < numOfIndices; j++)
            int idx
            fin >> idx
            chain.AddIndex(idx)

        m_chains.push_back(chain)


    return True


def FindNodeAccordingToName(self, str):
    for (i = 0; i < GetNumOfNodes(); i++)
        if GetNode(i).GetName() == std.string(str):
            return i


    std.cout << "Cannot find a node named " << str << "not \n"
    return -1


def RemoveIndividualNodes(self):
    numOfNodes = GetNumOfNodes()
    std.vector<bool> vecKeepFlags(numOfNodes, False)
    std.vector<int> vecNewIndices(numOfNodes, -1)
    std.vector<CGraphNode> nodesToKeep
    keepCount = 0
    for (i = 0; i < numOfNodes; i++)
        node = GetNode(i)
        if node.GetNeighbors().empty() == False:
            nodesToKeep.push_back(node)
            vecKeepFlags[i] = True
            vecNewIndices[i] = keepCount
            keepCount++


    if keepCount == numOfNodes:
        return

    m_nodes = nodesToKeep
    for (i = 0; i < GetNumOfEdges(); i++)
        edge = GetEdge(i)
        idx0 = edge.GetIdx0()
        idx1 = edge.GetIdx1()
        idx0 = vecNewIndices[idx0]
        idx1 = vecNewIndices[idx1]
        edge.SetIdx0(idx0)
        edge.SetIdx1(idx1)


    SetNodeNeighbors()

