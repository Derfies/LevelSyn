#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-03-07
#  email:  chongyangm@gmail.com
#  info: class declaration of the level synthesis algorithm
# --------------------------------------------------------------

#ifndef LEVELSYNTH_H
#define LEVELSYNTH_H

#include "ConfigSpace.h"
#include "LevelConfig.h"
#include "PlanarGraph.h"
#include "RoomLayout.h"
#include "RoomTemplates.h"
#include "clipperWrapper.h"

#include <stack>
#include <string>

#ifdef __linux__
    #include <stdarg.h>
#endif

##define PRINT_OUT_DEBUG_INFO

# Use to track current solution state
class CurrentState
public:
    CPlanarGraph m_stateGraph
    std.vector<v2f> m_stateRoomPositions
    std.vector<int> myIndices

    float m_stateEnergy

    void MoveRoomsToSceneCenter(CPlanarGraph* ptrGraph)

    void Move1DchainToSceneCenter(std.vector<int>& indices)

    float GetStateDifference(CurrentState& otherState, ptrGraph)

    bool InsertToNewStates(std.vector<CurrentState>& newStates, ptrGraph)


class CLevelSynth
public:
    CLevelSynth()

    CLevelSynth(CPlanarGraph* ptrGraph, ptrTemplates)

    void SetGraphAndTemplates(CPlanarGraph* ptrGraph, ptrTemplates)

    void SetGraph(CPlanarGraph* ptrGraph)

    bool MovePickedGraphNode(float& dx, dy)

    bool AdjustPickedRoom(float& dx, dy)

    void InitScene()

    CRoomLayout GetLayout(CPlanarGraph* ptrGraph, roomPositions)

    void SynthesizeScene()

    void UpdateGraphFromLayout()

    bool PostProcessing(CRoomLayout& layout, ptrGraph)

    bool OpenDoors(CRoomLayout& layout, ptrGraph, flagPartial = False)

    bool OpenDoor(CRoom& room, door, width = -1.f)

    bool OpenDoors(CRoomLayout& layout, layoutShrinked, ptrGraph, thrinkDist)

    void ShrinkRooms(CRoomLayout& layout, dist)

    void ShrinkRoom(CRoom& room, dist)

    bool SaveGraphAsSVG( char* fileName, ptrGraph, wd = 800, ht = 800, labelRad = 0.25f)

    static bool CompareStateEnergySmallerFirst( CurrentState& state1, state2)

    int GetSolutionCount() { return m_solutionCount;

    void ResetSolutionCount() { m_solutionCount = 0;

    void ResetIterationCount()
        m_chainCount = 0


    inline std.string sprint( char* fmt, ...)
        size = 512
        buffer = 0
        buffer = char[size]
        va_list vl
        va_start(vl, fmt)
        nsize = vsnprintf(buffer, size, fmt, vl)
        if size <= nsize:
            #fail delete buffer and try again
            delete[] buffer
            buffer = 0
            buffer = char[nsize + 1]; #+1 for /0
            nsize = vsnprintf(buffer, size, fmt, vl)

        std.string ret(buffer)
        va_end(vl)
        delete[] buffer
        return ret


private:
    void SynthesizeSceneViaMainLoop()

    bool Solve1Dchain(std.vector<int>& indices, tmpIndices, oldState, newStates)

    bool Solve1DchainILS(std.vector<int>& indices, oldState, newStates)

    void SetCurrentState(CurrentState& s)

    void SetSequenceAs1Dchain( std.vector<int>& indices, ptrGraph)

    void SetVisitedNeighbors( std.vector<int>& indices)

    void DumpSolutionIntoXML()

    int RandomlyPickOneRoom(CRoomLayout& layout)

    int RandomlyPickOneRoom(std.vector<int>& indices, weightedIndices = NULL)

    int RandomlyPickOneRoom(CRoomLayout& layout, indices, weightedIndices)

    int RandomlyPickAnotherRoom(CRoomLayout& layout, pickedIndex)

    std.vector<int> GetConnectedIndices(CPlanarGraph* ptrGraph, pickedIndex, flagVisitedOnly = True)

    int RandomlyAdjustOneRoom(CRoomLayout& layout, ptrGraph, indices, weightedIndices)

    void RandomlyAdjustOneRoom01(CRoomLayout& layout, ptrGraph, indices)

    void RandomlyAdjustOneRoom02(CRoomLayout& layout, ptrGraph, indices)

    int RandomlyAdjustOneRoom03(CRoomLayout& layout, ptrGraph, indices, weightedIndices)

    void SampleConfigSpaceForPickedRoom(CRoomLayout& layout, ptrGraph, indices, pickedRoomIndex)

    int RandomlyAdjustOneRoom04(CRoomLayout& layout, ptrGraph, indices, weightedIndices)

    int GradientDescentOneRoom(CRoomLayout& layout, ptrGraph, indices)

    float GetLayoutEnergy(CRoomLayout& layout, ptrGraph, collideArea, connectivity, roomThatMoved = -1, doContact = False, indicesForContact = NULL)

    bool GetLayoutEnergyEarlyOut(CRoomLayout& layout, ptrGraph, collideArea, connectivity, roomThatMoved = -1, energyTmp = NULL, energyCurrent = 0.0f)

    float CheckRoomConnectivity(CRoomLayout& layout, ptrGraph, flagVisitedOnly = False, roomThatMoved = -1)

    float LayoutCollide(CRoomLayout& layout, ptrGraph, flagVisitedOnly = False, roomThatMoved = -1)

    float LayoutCollide(CRoomLayout& layout)

    float RoomCollides(CRoom& room1, room2)

    float BoundingBoxCollidesArea(AABB2f& bb1, bb2); # not-in-use

    bool TestBoundingBoxCollides(AABB2f& bb1, bb2)

    float LayoutContact(CRoomLayout& layout, ptrGraph, flagVisitedOnly = False, flagNonOverlap = False, indices = NULL, roomThatMoved = -1)

    v2f ComputeLabelPosition(int idx, ptrGraph, labelRad)

    std.vector<int> m_sequence; # 1D chain of instantiated room templates

    CPlanarGraph* m_ptrGraph
    CRoomTemplates* m_ptrTemplates
    CRoomLayout m_layout

    int m_solutionCount
    std.vector<v2f> m_roomPositions
    std.vector<std.vector<int>> m_visitedNeighbors
    m_bestSolCount = 0

    bool m_flagVisitedNoNode

    int m_chainCount

    int m_backTrackCount
    int m_backTrackLevel


#endif # LEVELSYNTH_H
