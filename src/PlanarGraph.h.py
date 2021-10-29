#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-02-28
#  email:  chongyangm@gmail.com
#  info: class declaration of a planar graph
# --------------------------------------------------------------

#ifndef PLANARGRAPH_H
#define PLANARGRAPH_H

#include "tinyxml2.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/ref.hpp>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
using namespace boost

#include "GraphChain.h"
#include "GraphEdge.h"
#include "GraphFace.h"
#include "GraphNode.h"
#include "vec.h"

class CPlanarGraph
public:
    typedef adjacency_list<vecS,
                           vecS,
                           undirectedS,
                           property<vertex_index_t, int>,
                           property<edge_index_t, int>>
        Graph

    typedef graph_traits<Graph>.vertex_descriptor VertexDescriptor
    typedef graph_traits<Graph>.vertex_iterator VertexIterator
    typedef graph_traits<Graph>.edge_descriptor EdgeDescriptor
    typedef graph_traits<Graph>.out_edge_iterator OutEdgeIterator

    CPlanarGraph()

    void ClearGraph()

    void PrintGraph()

    bool LoadGraphFromXML( char* fileName, flagDetectFaces = True, flagIgnoreIndiv = True)

    bool SaveGraphAsXML( char* fileName)

    void AddGraphNode(CGraphNode& node) { m_nodes.push_back(node);
    #void AddGraphEdge(CGraphEdge& edge) { m_edges.push_back(edge);
    bool AddGraphEdge( CGraphEdge& edge)

    bool CheckDuplicatedEdge( CGraphEdge& edge)

    void AddGraphNodes(int numOfNodes, parent = -1)

    void SetNodeNeighbors()

    bool FindLineIntersect(v2f p1, p2, p3, p4)
        return FindLineIntersect(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1])


    bool FindLineIntersect(float x1, y1, x2, y2, x3, y3, x4, y4)
        s1 = (y1 - y2) * (x3 - x4) - (x1 - x2) * (y3 - y4)
        s0 = (x1 - x2) * (y4 - y2) - (y1 - y2) * (x4 - x2)
        t1 = (y3 - y4) * (x1 - x2) - (x3 - x4) * (y1 - y2)
        t0 = (x3 - x4) * (y2 - y4) - (y3 - y4) * (x2 - x4)
        if std.abs(s1) > 0.f and std.abs(t1) > 0.f:
            s = s0 / s1
            t = t0 / t1
            if s > 0.f and s < 1.f and t > 0.f and t < 1.f:
                return True



        return False


    void PickClosestNode(float px, py)

    void MovePickedNode(float dx, dy)

    void MovePickedNode(v2f dp) { MovePickedNode(dp[0], dp[1]);

    void UnpickNode() { m_pickedNodeIndex = -1;

    int GetPickedNodeIndex() { return m_pickedNodeIndex;

    int GetNumOfNodes() { return int(m_nodes.size());

    int GetNumOfEdges() { return int(m_edges.size());

    int GetNumOfFaces() { return int(m_faces.size());

    v2f GetNodePos(int idx) { return m_nodes[idx].GetPos();

    void RandomInitGraph()

    void RandomInitPositions()

    void RandomInitTypes()

    void DetectFaces()

    void SortAdjacentVertices( Graph& g, vert, adjacentEdges)

    bool CompareEdgeDirections( v2f& edgePr1, edgePr2, edgeRef)

    void RemoveTheOutsideFace()

    CGraphNode& GetNode(int idx) { return m_nodes[idx];
    CGraphEdge& GetEdge(int idx) { return m_edges[idx];
    CGraphFace& GetFace(int idx) { return m_faces[idx];
    CGraphChain& GetChain(int idx) { return m_chains[idx];

    bool VisitedAllNodes()

    bool VisitedNoNode()

    bool HasFixedNode()

    std.vector<int> GetFixedNodes()

    std.vector<int> GetUnfixedNodes()

    # Step 3: Extract the 'deepest' face or chain not yet inserted (the most included one)
    std.vector<int> ExtractDeepestFaceOrChain(bool& flagCyclic, flagSmallFaceFirst)

    std.vector<int> ExtractDeepestFaceOrChainOld(bool& flagCyclic, flagSmallFaceFirst)

    # Extract the 'deepest' face not yet inserted
    std.vector<int> ExtractDeepestFace(bool flagSmallFaceFirst)

    # Extract the 'deepest' chain not yet inserted
    std.vector<int> ExtractDeepestChainNew()

    std.vector<int> ExtractDeepestChain()

    int CountConstraints(std.vector<int>& indices)

    void SetNumOfTypes(int numOfTypes) { m_numOfTypes = numOfTypes;

    void GetGraphBoundingBox(v2f& posMin, posMax)

    void MoveGraphToSceneCenter()

    void ScaleGraphNodePositions(float scaling)

    bool LoadChainsFromTXT( char* fileName)

    int FindNodeAccordingToName( char* str)

    void RemoveIndividualNodes()

private:
    inline int Random2(int max)
        if max < 1 or max >= RAND_MAX:
            return 0
        else:
            return (int)rand() / (RAND_MAX / max + 1)


    inline bool OneChanceIn(int a_million)
        return (Random2(a_million) == 0)


    inline bool CoinFlip()
        return OneChanceIn(2)


    std.vector<CGraphNode> m_nodes
    std.vector<CGraphEdge> m_edges
    int m_pickedNodeIndex

    static CGraphFace m_faceTmp
    static std.vector<CGraphFace> m_faces

    std.vector<CGraphChain> m_chains

    int m_numOfTypes; # number of node types

    # Some planar face traversal visitors that will
    # print the vertices and edges on the faces
    struct output_visitor : public planar_face_traversal_visitor
        void begin_face()
            m_faceTmp.ClearIndices()
            std.cout << "New face: "


        void end_face()
            std.cout << std.endl
            if m_faceTmp.IsEmpty() == False:
                m_faces.push_back(m_faceTmp)




    struct vertex_output_visitor : public output_visitor
        template <typename Vertex>
        void next_vertex(Vertex v)
            std.cout << v << " "
            m_faceTmp.AddIndex(int(v))



    struct edge_output_visitor : public output_visitor
        template <typename Edge>
        void next_edge(Edge e)
            std.cout << e << " "




#endif # PLANARGRAPH_H
