import copy
import random
import math

from dataclasses import dataclass

from clipperwrapper import compute_collide_area
from configspace import ConfigSpace
from levelconfig import LevelConfig
from roomlayout import RoomLayout
from reactor.geometry.vector import Vector2
from levelmath import NUMERICAL_TOLERANCE, room_contact, room_distance


@dataclass
class AABB2:

    pos_min: Vector2
    pos_max: Vector2


def random2(max_):
    if max_ < 1:
        return 0
    else:
        return random.randint(0, max_ - 1)


class CurrentState:
    
    def __init__(self):
        self.state_room_positions = []
        self.my_indices = []
        self.state_energy = 0

    def move_rooms_to_scene_centre(self, graph):
        p_min = Vector2(1e10, 1e10)
        p_max = Vector2(-1e10, -1e10)
        for j in range(len(graph.nodes)):
            if not graph.get_node(j).flag_visited:
                continue

            pj = self.state_room_positions[j]
            for k in range(2):
                p_min[k] = min(p_min[k], pj[k])
                p_max[k] = max(p_max[k], pj[k])

        pos_cen = (p_min + p_max) * 0.5
        for j in range(len(graph.nodes)):
            self.state_room_positions[j] = self.state_room_positions[j] - pos_cen

    def move_1d_chain_to_scene_centre(self, indices):
        p_min = Vector2(1e10, 1e10)
        p_max = Vector2(-1e10, -1e10)
        for j in range(len(indices)):
            idx = indices[j]
            pj = self.state_room_positions[idx]
            for k in range(2):
                p_min[k] = min(p_min[k], pj[k])
                p_max[k] = max(p_max[k], pj[k])

        pos_cen = (p_min + p_max) * 0.5
        for j in range(len(indices)):
            idx = indices[j]
            self.state_room_positions[idx] = self.state_room_positions[idx] - pos_cen

    def get_state_difference(self, other_state, graph):
        state_diff = 0
        for j in range(graph.num_nodes):
            if not graph.get_node(j).flag_visited:
                continue
            p1 = self.state_room_positions[j]
            p2 = other_state.state_room_positions[j]
            state_diff += (p1 - p2).mag2()
        return state_diff

    def insert_to_new_states(self, new_states, graph):
        state_diff_thresh = LevelConfig().STATE_DIFFERENCE_THRESHOLD
        if state_diff_thresh <= 0:
            new_states.append(self)
            return True

        for i in range(len(new_states)):
            if self.state_energy < new_states[i].state_energy:
                continue

            state_diff = self.get_state_difference(new_states[i], graph)
            if state_diff <= state_diff_thresh:
                return False

        new_states.append(self)
        return True
 
        
class LevelSynth:
    
    def __init__(self):
        self.graph = None
        self.templates = None
        self.solution_count = 0
        self.bestSolCount = 0
        self.chain_count = 0
        self.backtrack_count = 0
        self.backtrack_level = 0
        self.layout = RoomLayout()
        self.visited_neighbours = []
        self.sequence = []

    def set_graph_and_templates(self, graph, templates):
        self.solution_count = 0
        self.best_sol_count = 0
        graph.move_graph_to_scene_centre()
        graph.scale_graph_node_positions(LevelConfig().GRAPH_SCALING)
        self.set_graph(graph)
        self.templates = templates
        self.graph.num_types = self.templates.num_templates
        self.graph.random_init_types()
        self.init_scene()
        self.synthesize_scene()

    def set_graph(self, graph):
        self.graph = graph
        self.room_positions = []
        for i in range(self.graph.num_nodes):
            pi = self.graph.get_node_pos(i)
            self.room_positions.append(pi)
    #
    # def MovePickedGraphNode(self, dx, dy):
    #     self.graph.move_picked_node(dx, dy)
    #     self.init_scene()
    #     flag = False
    #     #flag = adjust_picked_room(dx, dy)
    #     return flag
    #
    # def adjust_picked_room(self, dx, dy):
    #     picked_node_index = self.graph.get_picked_node_index()
    #     flag = False
    #     if picked_node_index < 0:
    #         return flag
    #
    #     picked_room = self.layout.get_room(picked_node_index)
    #     for (i = 0; i < picked_room.GetNumOfEdges(); i++):
    #         edge = picked_room.get_edge(i)
    #         pr2 = edge.GetPos2() - edge.GetPos1()
    #         pr = Vector3(pr2[0], pr2[1], 0.f)
    #         norm = Vector2(pr[1], -pr[0])
    #         norm = norm.normalize()
    #         distMin = 1e10
    #         CRoomEdge edgeNearest
    #         for (j = 0; j < self.layout.Getnum_rooms(); j++):
    #             if j == picked_node_index:
    #                 continue
    #
    #             other_room = self.layout.get_room(j)
    #             for (k = 0; k < other_room.GetNumOfEdges(); k++):
    #                 otherEdge = other_room.get_edge(k)
    #                 otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
    #                 otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
    #                 cp = cross(pr, otherPr)
    #                 if mag2(cp) > 0.0001f:
    #                     continue
    #
    #                 prTmp = otherEdge.GetPos1() - edge.GetPos1()
    #                 distTmp = std.abs(dot(norm, prTmp))
    #                 if distTmp < distMin:
    #                     distMin = distTmp
    #                     edgeNearest = otherEdge
    #
    #         if distMin < 0.05f:
    #             pr = edgeNearest.GetPos1() - edge.GetPos1()
    #             d = dot(norm, pr)
    #             dp = d * norm
    #             self.graph.move_picked_node(dp)
    #             init_scene()
    #             dx += dp[0]
    #             dy += dp[1]
    #             flag = True
    #
    #     return flag
    #
    def init_scene(self):
        self.layout.clear_layout()
        #num_rooms =
        num_templates = self.templates.num_templates
        for i in range(self.graph.num_nodes):
            #idx = int(rand() / float(RAND_MAX) * num_templates)
            idx = self.graph.get_node(i).type % num_templates
            #idx = idx
            room = self.templates.rooms[idx]
            #room.ScaleRoom(0.5f)
            pi = self.graph.get_node_pos(i)
            c = room.get_room_centre()
            trans = pi - c
            room.translate_room(trans)
            # color = randomColorFromIndex(i)
            # if mag2(color) > 2.5f:
            #     color = color * 0.5f
            # room.SetColor(color)
            self.layout.rooms.append(copy.deepcopy(room))

    def get_layout(self, graph, room_positions):
        layout = RoomLayout()
        num_rooms = graph.num_nodes
        num_templates = self.templates.num_templates
        #for (i = 0; i < num_rooms; i++):
        for i in range(num_rooms):
            idx = graph.get_node(i).type
            idx = idx % num_templates
            room = self.templates.rooms[idx]
            pi = room_positions[i]
            c = room.get_room_centre()
            trans = pi - c
            room.translate_room(trans)
           # color = randomColorFromIndex(i)
           # if mag2(color) > 2.5f:
           #     color = color * 0.5f

            #room.SetColor(color)
            room.flag_fixed = graph.get_node(i).flag_fixed
            layout.rooms.append(room)

        return layout

    def synthesize_scene(self):
        self.synthesize_scene_via_main_loop()
        self.update_graph_from_layout()

    def update_graph_from_layout(self):
        num_rooms = self.graph.num_nodes
        for i in range(num_rooms):
            room_centre = self.layout.rooms[i].get_room_centre()
            self.graph.get_node(i).pos = room_centre
    #
    # def PostProcessing(self, layout, graph):
    #     for (i = 0; i < layout.Getnum_rooms(); i++):
    #         std.vector<int> neighbors
    #         for (j = 0; j < graph.GetNumOfEdges(); j++):
    #             edge = graph.get_edge(j)
    #             if edge.idx0 == i:
    #                 neighbors.push_back(edge.idx1)
    #
    #             elif edge.idx1 == i:
    #                 neighbors.push_back(edge.idx0)
    #
    #
    #         if neighbors.empty() == True:
    #             continue
    #
    #         Cconfig_space config_space0(layout.get_room(neighbors[0]), layout.get_room(i))
    #         for (j = 1; j < int(neighbors.size()); j++):
    #             Cconfig_space config_space1(layout.get_room(neighbors[j]), layout.get_room(i))
    #             config_space0 = Cconfig_space.find_intersection(config_space0, config_space1)
    #
    #         config_space0.SelfMerge()
    #         print(f'Size of configuration space for the {i}th room: {config_space0.Getconfig_spaceSize()}')
    #         config_space0.Printconfig_space()
    #
    #     return True
    #
    # def open_doors(self, layout, graph, flag_partial ''' = False '''):
    #     for (i = 0; i < layout.Getnum_rooms(); i++):
    #         layout.get_room(i).InitWalls()
    #
    #     for (i = 0; i < graph.GetNumOfEdges(); i++):
    #         ge = graph.get_edge(i)
    #         roomIdx1 = ge.idx0
    #         roomIdx2 = ge.idx1
    #         room1 = layout.get_room(roomIdx1)
    #         room2 = layout.get_room(roomIdx2)
    #         if room1.get_flag_fixed() or room2.get_flag_fixed():
    #             continue
    #
    #         int edgeIdx1, edgeIdx2
    #         contact = room_contact(room1, room2, edgeIdx1, edgeIdx2)
    #         if (contact < ROOM_CONTACT_THRESH): # just to double check
    #             if not flag_partial:
    #                 print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 1)!')
    #                 return False
    #             else:
    #                 continue
    #
    #         std.vector<v2f> vec_pos(4)
    #         edge1 = room1.get_edge(edgeIdx1)
    #         edge2 = room2.get_edge(edgeIdx2)
    #         vec_pos[0] = edge1.GetPos1()
    #         vec_pos[1] = edge1.GetPos2()
    #         vec_pos[2] = edge2.GetPos1()
    #         vec_pos[3] = edge2.GetPos2()
    #         SortVecPr(vec_pos)
    #         RoomDoor door(vec_pos[1], vec_pos[2])
    #         flag1 = OpenDoor(room1, door)
    #         flag2 = OpenDoor(room2, door)
    #         if not flag1 or not flag2:
    #             print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 2)!')
    #             return False
    #
    #     return True
    #
    # def OpenDoor(self, room, door, width ''' = -1.f '''):
    #      numericalTolerance = self.numerical_tolerance * 100.f
    #      numericalToleranceSq = numericalTolerance * numericalTolerance
    #      doorWidth = (width > 0.f) ? width : (ROOM_CONTACT_THRESH * 0.8f)
    #     openFlag = False
    #     for (i = 0; i < room.GetNumOfWalls(); i++)
    #         wall = room.GetWall(i)
    #         d1 = PointToSegmentSqDistance(door.GetPos1(), wall)
    #         d2 = PointToSegmentSqDistance(door.GetPos2(), wall)
    #         if d1 > numericalToleranceSq or d2 > numericalToleranceSq:
    #             continue
    #
    #         room.EraseWall(i)
    #         d11 = mag2(door.GetPos1() - wall.GetPos1())
    #         d12 = mag2(door.GetPos2() - wall.GetPos1())
    #         p1 = (d11 < d12) ? door.GetPos1() : door.GetPos2()
    #         p2 = (d11 < d12) ? door.GetPos2() : door.GetPos1()
    #         pMean = (p1 + p2) * 0.5f
    #         pd = p2 - p1
    #         pd = normalize(pd) * doorWidth * 0.5f
    #         p1 = pMean - pd
    #         p2 = pMean + pd
    #         RoomWall wall1(wall.GetPos1(), p1)
    #         RoomWall wall2(p2, wall.GetPos2())
    #         room.InsertWall(wall1)
    #         room.InsertWall(wall2)
    #         openFlag = True
    #         break
    #
    #     return openFlag
    #
    # def open_doors(self, layout, layoutShrinked, graph, thrinkDist):
    #     for (i = 0; i < layout.Getnum_rooms(); i++):
    #         layout.get_room(i).InitWalls()
    #         layoutShrinked.get_room(i).InitWalls()
    #
    #     doorWidth = ROOM_CONTACT_THRESH * 0.8f
    #     for (i = 0; i < graph.GetNumOfEdges(); i++):
    #         ge = graph.get_edge(i)
    #         roomIdx1 = ge.idx0
    #         roomIdx2 = ge.idx1
    #         room1 = layout.get_room(roomIdx1)
    #         room2 = layout.get_room(roomIdx2)
    #         if room1.get_flag_fixed() or room2.get_flag_fixed:
    #             continue
    #
    #         int edgeIdx1, edgeIdx2
    #         contact = room_contact(room1, room2, edgeIdx1, edgeIdx2)
    #         if (contact < doorWidth) # just to double check
    #             print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 1)!')
    #             return False
    #
    #         std.vector<v2f> vec_pos(4)
    #         edge1 = room1.get_edge(edgeIdx1)
    #         edge2 = room2.get_edge(edgeIdx2)
    #         vec_pos[0] = edge1.GetPos1()
    #         vec_pos[1] = edge1.GetPos2()
    #         vec_pos[2] = edge2.GetPos1()
    #         vec_pos[3] = edge2.GetPos2()
    #         SortVecPr(vec_pos)
    #         p1 = vec_pos[1]
    #         p2 = vec_pos[2]
    #         pr = p2 - p1
    #         pr = normalize(pr)
    #         prOrtho = v2f(pr[1], -pr[0])
    #         pr = pr * doorWidth * 0.5f
    #         pAve = (p1 + p2) * 0.5f
    #         p1 = pAve - pr
    #         p2 = pAve + pr
    #         p3 = p1 + prOrtho * thrinkDist
    #         p4 = p1 - prOrtho * thrinkDist
    #         p5 = p2 + prOrtho * thrinkDist
    #         p6 = p2 - prOrtho * thrinkDist
    #         CorridorWall wall1(p3, p4)
    #         CorridorWall wall2(p5, p6)
    #         layoutShrinked.InsertCorridorWall(wall1)
    #         layoutShrinked.InsertCorridorWall(wall2)
    #         RoomDoor door1(p3, p5)
    #         RoomDoor door2(p4, p6)
    #         flag1 = OpenDoor(layoutShrinked.get_room(roomIdx1), door1, doorWidth) or OpenDoor(layoutShrinked.get_room(roomIdx1), door2, doorWidth)
    #         flag2 = OpenDoor(layoutShrinked.get_room(roomIdx2), door1, doorWidth) or OpenDoor(layoutShrinked.get_room(roomIdx2), door2, doorWidth)
    #         if flag1 == False or flag2 == False:
    #             print(f'Failed to open the door on the wall between Room {roomIdx1} and Room {roomIdx2} (case 2)!')
    #             return False
    #
    #     return True
    #
    # # def ShrinkRooms(self, layout, dist):
    # #     if dist <= 0.f:
    # #         return
    # #
    # #     for (i = 0; i < layout.Getnum_rooms(); i++):
    # #         ShrinkRoom(layout.get_room(i), dist)
    # #
    # # def ShrinkRoom(self, room, dist):
    # #     if dist <= 0.f:
    # #         return
    # #
    # #      numOfEdges = room.GetNumOfEdges()
    # #     std.vector<CRoomEdge> vecRoomEdge(numOfEdges)
    # #     for (i = 0; i < numOfEdges; i++):
    # #         vecRoomEdge[i] = room.get_edge(i)
    # #
    # #     std.vector<CRoomEdge> vecRoomEdge1 = vecRoomEdge
    # #     std.vector<CRoomEdge> vecRoomEdge2 = vecRoomEdge
    # #     for (i = 0; i < numOfEdges; i++):
    # #         pr = vecRoomEdge[i].GetPos2() - vecRoomEdge[i].GetPos1()
    # #         prNew = v2f(pr[1], -pr[0])
    # #         prNew = normalize(prNew) * dist
    # #         vecRoomEdge1[i].set_pos1(vecRoomEdge1[i].GetPos1() + prNew)
    # #         vecRoomEdge1[i].set_pos2(vecRoomEdge1[i].GetPos2() + prNew)
    # #         vecRoomEdge2[i].set_pos1(vecRoomEdge2[i].GetPos1() - prNew)
    # #         vecRoomEdge2[i].set_pos2(vecRoomEdge2[i].GetPos2() - prNew)
    # #
    # #     std.vector<v2f> vertices1 = room.GetVertices()
    # #     std.vector<v2f> vertices2 = room.GetVertices()
    # #     for (i = 0; i < numOfEdges; i++):
    # #         idx1 = i
    # #         idx2 = (i + 1) % numOfEdges
    # #         Vector2pi1
    # #         p111 = vecRoomEdge1[idx1].GetPos1()
    # #         p112 = vecRoomEdge1[idx1].GetPos2()
    # #         p121 = vecRoomEdge1[idx2].GetPos1()
    # #         p122 = vecRoomEdge1[idx2].GetPos2()
    # #         LineIntersection(p111, p112, p121, p122, pi1)
    # #         vertices1[i] = pi1
    # #         Vector2pi2
    # #         p211 = vecRoomEdge2[idx1].GetPos1()
    # #         p212 = vecRoomEdge2[idx1].GetPos2()
    # #         p221 = vecRoomEdge2[idx2].GetPos1()
    # #         p222 = vecRoomEdge2[idx2].GetPos2()
    # #         LineIntersection(p211, p212, p221, p222, pi2)
    # #         vertices2[i] = pi2
    # #
    # #     CClipperWrapper wrapper
    # #     CRoom room1
    # #     room1.SetVertices(vertices1)
    # #     area1 = wrapper.ComputeRoomArea(room1)
    # #     CRoom room2
    # #     room2.SetVertices(vertices2)
    # #     area2 = wrapper.ComputeRoomArea(room2)
    # #     std.vector<v2f> verticesNew = (area1 < area2) ? vertices1 : vertices2
    # #     room.SetVertices(verticesNew)
    #
    #
    # # def SaveGraphAsSVG(self, fileName, graph, wd ''' = 400 ''', ht ''' = 400 ''', labelRad ''' = 0.25f '''):
    # #     graphNew = *graph
    # #     graphNew.move_graph_to_scene_centre()
    # #     strokeWd = 5
    # #     circleRad = 7
    # #     Vector2posMin, posMax
    # #     graphNew.GetGraphBoundingBox(posMin, posMax)
    # #     p_min = min(posMin[0], posMin[1])
    # #     p_max = max(posMax[0], posMax[1])
    # #     scaling = 1.05f
    # #     p_min *= scaling
    # #     p_max *= scaling
    # #      str = '\t<?xml version=\'1.0\' standalone=\'no\' ?>\n'
    # #                       '<not -- graph visualization -.\n'
    # #                       '<svg>\n'
    # #                       '</svg>\n'
    # #     tinyxml2.XMLDocument doc
    # #     doc.Parse(str)
    # #     root = doc.RootElement()
    # #     std.ostringstream ossViewBox
    # #     ossViewBox << 0} {0} {wd} {ht
    # #     root.SetAttribute('viewBox', ossViewBox.str().c_str())
    # #     root.SetAttribute('xmlns', 'http:#www.w3.org/2000/svg')
    # #     # Dump edges...
    # #     for (i = 0; i < graphNew.GetNumOfEdges(); i++):
    # #         edgeElement = doc.NewElement('path')
    # #         std.ostringstream ossPath
    # #         edge = graphNew.get_edge(i)
    # #         p1 = graphNew.get_node_pos(edge.idx0)
    # #         p2 = graphNew.get_node_pos(edge.idx1)
    # #         ossPath}M '
    # #         ossPath << CRoomLayout.ConvertPosX(p1[0], p_min, p_max, wd)} '
    # #         ossPath << CRoomLayout.ConvertPosY(p1[1], p_min, p_max, ht)} '
    # #         ossPath}L '
    # #         ossPath << CRoomLayout.ConvertPosX(p2[0], p_min, p_max, wd)} '
    # #         ossPath << CRoomLayout.ConvertPosY(p2[1], p_min, p_max, ht)} '
    # #         edgeElement.SetAttribute('d', ossPath.str().c_str())
    # #         edgeElement.SetAttribute('fill', 'none')
    # #         edgeElement.SetAttribute('stroke', 'black')
    # #         edgeElement.SetAttribute('stroke-width', strokeWd)
    # #         root.InsertEndChild(edgeElement)
    # #
    # #     # Dump nodes...
    # #     for (i = 0; i < graphNew.num_nodes; i++):
    # #         nodeElement = doc.NewElement('circle')
    # #         pi = graphNew.get_node_pos(i)
    # #         nodeElement.SetAttribute('cx', CRoomLayout.ConvertPosX(pi[0], p_min, p_max, wd))
    # #         nodeElement.SetAttribute('cy', CRoomLayout.ConvertPosY(pi[1], p_min, p_max, ht))
    # #         nodeElement.SetAttribute('r', circleRad)
    # #         nodeElement.SetAttribute('fill', 'red')
    # #         nodeElement.SetAttribute('stroke', 'none')
    # #         root.InsertEndChild(nodeElement)
    # #
    # #     # Dump labels...
    # #     for (i = 0; i < graphNew.num_nodes; i++):
    # #         shiftX = (i >= 10) ? 8 : 3
    # #         shiftY = 5
    # #         pi = ComputeLabelPosition(i, &graphNew, labelRad)
    # #         labelElement = doc.NewElement('text')
    # #         labelElement.SetAttribute('x', CRoomLayout.ConvertPosX(pi[0], p_min, p_max, wd) - shiftX)
    # #         labelElement.SetAttribute('y', CRoomLayout.ConvertPosY(pi[1], p_min, p_max, ht) + shiftY)
    # #         labelElement.SetAttribute('font-family', 'Verdana')
    # #         labelElement.SetAttribute('font-size', 13)
    # #         labelElement.SetAttribute('fill', 'blue')
    # #         std.ostringstream ossLabel
    # #         ossLabel << i
    # #         labelText = doc.NewText(ossLabel.str().c_str())
    # #         labelElement.InsertEndChild(labelText)
    # #         root.InsertEndChild(labelElement)
    # #
    # #
    # #     saveFlag = doc.SaveFile(fileName)
    # #     return saveFlag
    #
    # def CompareStateEnergySmallerFirst(self, state1, state2):
    #     return state1.state_energy < state2.state_energy
    #
    def synthesize_scene_via_main_loop(self):
        state0 = CurrentState()
        state0.state_graph = self.graph
        state0.state_room_positions = self.room_positions[:]
        state0.state_energy = 1e10
        state_stack = []
        state_stack.insert(0, state0)
        target_num_solutions = LevelConfig().TARGET_NUM_SOLUTIONS
        energy_min = 1e10
        layout_best = self.layout
        num_partials = 0
        self.backtrack_count = 0
        self.backtrack_level = 0
        while self.solution_count < target_num_solutions and state_stack:
            old_state = state_stack.pop(0)
            self.set_current_state(old_state)
            self.flag_visited_no_node = self.graph.visited_no_node()
            flag_cyclic = False
            tmp_indices = self.graph.extract_deepest_face_or_chain(flag_cyclic, LevelConfig().FLAG_SMALL_FACE_FIRST)
            #indices = []
    #if 0 # Before 09/03/2013
            # if LevelConfig().SYNTHESIS_METHOD != 0 :
            #     # Select all the graph nodes...
            #     indices.resize(self.graph.num_nodes)
            #     for i in range(len(indices)):
            #         indices[i] = i
    #else:
            indices = old_state.my_indices[:]
            if LevelConfig().SYNTHESIS_METHOD != 0:
                if not self.graph.has_fixed_node() or not self.graph.visited_no_node():
                    indices = self.graph.get_unfixed_nodes()

            #for i in range(len(tmp_indices)):
            #    indices.append(tmp_indices[i])
            indices.extend(tmp_indices)
    #endif
            self.set_visited_neighbours(indices)
            for i in range(self.graph.num_nodes):
                self.graph.get_node(i).flag_visited = False

            for i in range(len(indices)):
                #index = indices[i]
                self.graph.get_node(indices[i]).flag_visited = True

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

            if self.graph.visited_all_nodes():
                for i in range(len(new_states)):
                    if self.solution_count >= target_num_solutions:
                        break

                    self.set_current_state(new_states[i])
                    if new_states[i].state_energy < energy_min:
                        energy_min = new_states[i].state_energy
                        layout_best = self.layout

                    #float collide
                    #float connectivity
                    energy, collide_area, connectivity = self.get_layout_energy(self.layout, self.graph)

                    #float CLevelSynth.get_layout_energy(CRoomLayout& layout, graph, collide_area, connectivity)

                    flag_valid = self.layout_collide(self.layout) <= NUMERICAL_TOLERANCE and self.check_room_connectivity(self.layout, self.graph) <= NUMERICAL_TOLERANCE
                    if not flag_valid:
                        # Skip invalid solution...
                        continue

                    #DumpSolutionIntoXML()
                    self.solution_count += 1

            else:
                for i in reversed(range(len(new_states))):
                    new_states[i].my_indices = indices
                    #new_states[i].state_graph = *self.graph
                    graph_best = new_states[i].state_graph
                    for n in range(graph_best.num_nodes):
                        pn = new_states[i].state_room_positions[n]
                        graph_best.get_node(n).pos = pn

                    layout_best = self.get_layout(graph_best, new_states[i].state_room_positions)
                    #ofstream fout
    #ifdef DUMP_PARTIAL_SOLUTION
                    #graph_best.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'partial_%03d.xml', num_partials)).c_str())
                    #open_doors(layout_best, &graph_best, True)
                    #layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'partial_%03d.svg', num_partials)).c_str(), 800, 800, True, &graph_best)
    #endif
                    #self.best_sol_count ++
                    num_partials += 1
                    state_stack.insert(0, new_states[i])

        self.layout = layout_best
        self.layout.move_to_scene_centre()

    #ifndef PERFORMANCE_TEST
        print(f'Total # of backtracks: {self.backtrack_count}')
    #endif
    #
    def solve_1d_chain(self, indices, weighted_indices, old_state, new_states):
        if LevelConfig().FLAG_USE_ILS:
            return self.solve_1d_chainILS(indices, old_state, new_states)
    #
    #     graph = old_state.state_graph
    #     self.set_sequence_as_1d_chain(indices, graph)
    #     new_states.clear()
    #     if graph.get_node(indices[0]).get_flag_fixed():
    #         old_state.insert_to_new_states(new_states, graph)
    #         return True
    #
    #     flag_last_chain = graph.VisitedAllNodes()
    #
    #     # Number of cycles
    #     n = SA_NUM_CYCLES
    #     # Number of trials per cycle
    #     m = SA_NUM_TRIALS
    #     # Number of accepted solutions
    #     na = 1
    #     # Probability of accepting worse solution at the start
    #     p1 = SA_PROB1
    #     # Probability of accepting worse solution at the end
    #     p0 = SA_PROB0
    #     # Initial temperature
    #     t1 = -1.0 / math.log(p1)
    #     # Final temperature
    #     t0 = -1.0 / math.log(p0)
    #     # Fractional reduction every cycle
    #     frac = pow(t0 / t1, 1.0 / float(n - 1.0))
    #     # Current temperature
    #     t = t1
    #     # delta_e Average
    #     delta_e_avg = 0.0
    #     # Current best result so far
    #     layout_best = get_layout(graph, old_state.state_room_positions)
    #     graph_best = *graph
    #
    #     if not FLAG_RANDOM_WALK:
    # #if 0
    #         for (i = 0; i < weighted_indices.size(); i++):
    #             picked_room = layout_best.get_room((*weighted_indices)[i])
    #             sample_config_space_for_picked_room(layout_best, graph, indices, (*weighted_indices)[i])
    #
    # #else # New on 09/24/2013: connect the rooms together as initial guess
    #         num_visited_nodes = int(len(indices) - len(weighted_indices))
    #         if (num_visited_nodes == 0): # The first chain...
    #             num_visited_nodes = 1
    #
    #         std.vector<int> indicesVisited(num_visited_nodes)
    #         std.vector<bool> flagsVisited(graph.num_nodes, False)
    #         for (i = 0; i < num_visited_nodes; i++):
    #             indicesVisited[i] = indices[i]
    #             flagsVisited[indices[i]] = True
    #
    #         while (indicesVisited.size() < indices.size()):
    #             idxUnvisited = -1
    #             idxVisited = -1
    #             for (i = 0; i < int(weighted_indices.size()); i++):
    #                 idx = (*weighted_indices)[i]
    #                 if flagsVisited[idx]:
    #                     continue
    #
    #                 std.vector<int> connected_indices = get_connected_indices(graph, idx, False)
    #                 std.vector<int> visitedNeighbors
    #                 for (j = 0; j < int(connected_indices.size()); j++):
    #                     idxOther = connected_indices[j]
    #                     if flagsVisited[idxOther]:
    #                         visitedNeighbors.push_back(idxOther)
    #
    #                 if not visitedNeighbors.empty():
    #                     idxUnvisited = idx
    #                     room_unvisited = layout_best.get_room(idxUnvisited)
    #                     random_shuffle(visitedNeighbors.begin(), visitedNeighbors.end())
    #                     Cconfig_space config_space(layout_best.get_room(visitedNeighbors[0]), room_unvisited)
    #                     for (j = 1; j < int(visitedNeighbors.size()); j++):
    #                         Cconfig_space config_space_tmp(layout_best.get_room(visitedNeighbors[j]), room_unvisited)
    #                         config_space_new = Cconfig_space.find_intersection(config_space, config_space_tmp)
    #                         if config_space_new.IsEmpty():
    #                             break
    #                         else:
    #                             config_space = config_space_new
    #
    #
    #                     indicesVisited.push_back(idxUnvisited)
    #                     flagsVisited[idxUnvisited] = True
    #     #if 1 # Smartly sample the configuration space based on the energy...
    #                     std.vector<v2f> vec_pos = config_space.SmartlySampleconfig_space()
    #                     idx_best = -1
    #                     energy_min = 1e10
    #                     graph_tmp = *graph
    #                     for (j = 0; j < graph_tmp.num_nodes; j++):
    #                         graph_tmp.get_node(j).set_flag_visited(flagsVisited[j])
    #
    #                     for (j = 0; j < int(vec_pos.size()); j++):
    #                         layout_tmp = layout_best
    #                         picked_roomTmp = layout_tmp.get_room(idxUnvisited)
    #                         dp = vec_pos[j] - picked_roomTmp.get_room_centre()
    #                         picked_roomTmp.translate_room(dp)
    #                         collide_area = 0.f
    #                         connectivity = 0.f
    #                         energy_tmp = get_layout_energy(layout_tmp, &graph_tmp, collide_area, connectivity)
    #                         if collide_area < energy_min:
    #                             energy_min = collide_area
    #                             idx_best = j
    #
    #
    #                     if idx_best >= 0:
    #                         dp = vec_pos[idx_best] - room_unvisited.get_room_centre()
    #                         room_unvisited.translate_room(dp)
    #
    #     #else:
    #                     pos = config_space.randomly_sample_config_space()
    #                     dp = pos - room_unvisited.get_room_centre()
    #                     room_unvisited.translate_room(dp)
    #     #endif
    #                     break
    #
    # #endif
    #     self.layout = layout_best
    # #ifdef DUMP_INTERMEDIATE_OUTPUT
    #     layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'chainInit_%03d.svg', self.chain_count)).c_str())
    # #endif
    #     collide_area = 0.0
    #     connectivity = 0.0
    #     energy_min = self.get_layout_energy(layout_best, graph, collide_area, connectivity, -1, True, indices)
    #     energy_current = energy_min
    # #ifdef DUMP_INTERMEDIATE_OUTPUT
    #     print(f'Initial energy: {energy_min}')
    #     #std.ofstream fout
    #     ##fout.open(CLevelConfig.AddOutputPrefix('log.txt').c_str(), std.ios_base.app)
    #     #fout << self.best_sol_count}\t{energy_min << std.endl
    #     for (n = 0; n < graph.num_nodes; n++):
    #         pn = layout_best.get_room_positions()[n]
    #         graph.get_node(n).set_pos(pn)
    #
    #     graph.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.xml', self.best_sol_count)).c_str())
    #     open_doors(layout_best, graph, True)
    #     layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.svg', self.best_sol_count)).c_str())
    #     self.best_sol_count++
    # #endif
    #     pick_index_count = 0
    #     num_failures = 0
    #     for (i = 0; i < n; i++):
    # #ifndef PERFORMANCE_TEST
    #         print(f'Cycle {i + 1}/{n} (failures {num_failures}) ...\n')
    # #endif
    #         flag_was_accepted = False
    #         if num_failures > 10:
    #             if random2(2) == 0:
    # #ifndef PERFORMANCE_TEST
    #                 print(f'RANDOM RESTART CALLED! 11+ failures')
    # #endif
    #                 break
    #
    #         elif num_failures > 8:
    #             if random2(3) == 0:
    # #ifndef PERFORMANCE_TEST
    #                 print(f'RANDOM RESTART CALLED!  9+ failures')
    # #endif
    #                 break
    #
    #         elif num_failures > 5:
    #             if random2(4) == 0:
    # #ifndef PERFORMANCE_TEST
    #                 print(f'RANDOM RESTART CALLED!  6+ failures')
    # #endif
    #                 break
    #
    #         elif num_failures > 3:
    #             if random2(6) == 0:
    # #ifndef PERFORMANCE_TEST
    #                 print(f'RANDOM RESTART CALLED!  4+ failures')
    # #endif
    #                 break
    #
    #         elif num_failures > 0:
    #             if random2(8) == 0:
    # #ifndef PERFORMANCE_TEST
    #                 print(f'RANDOM RESTART CALLED: 1 failure but we just felt like quittin')
    # #endif
    #                 break
    #
    #         for (j = 0; j < m; j++):
    #             graph_tmp = graph
    #             layout_tmp = self.layout
    #             adjusted_index = randomly_adjust_one_room(layout_tmp, &graph_tmp, indices, weighted_indices)
    # #if 0 # New on 08/16/2013
    #             if self.flag_visited_no_node:
    #                 p_min = Vector2(1e10, 1e10)
    #                 p_max = Vector2(-1e10, -1e10)
    #                 for d in range(len(indices)):
    #                     idx = indices[d]
    #                     pj = layout_tmp.get_room(idx).get_room_centre()
    #                     for k in range(2):
    #                         p_min[k] = min(p_min[k], pj[k])
    #                         p_max[k] = max(p_max[k], pj[k])
    #
    #                 pos_cen = (p_min + p_max) * 0.5f
    #                 for d in range(len(indices)):
    #                     idx = indices[d]
    #                     layout_tmp.get_room(idx).translate_room(-pos_cen)
    #
    # #endif
    #             energy_tmp = get_layout_energy(layout_tmp, graph_tmp, collide_area, connectivity, adjusted_index, True, indices)
    #
    #             # accept = get_layout_energy_early_out(layout_tmp, &graph_tmp, collide_area, connectivity, adjusted_index, &energy_tmp, energy_current )
    #
    #             if ((FLAG_RANDOM_WALK and collide_area <= 1e-3 and connectivity <= 1e-3) or
    #                 (collide_area <= 1e-4 and connectivity <= 1e-4)):
    #                 new_state = old_state
    #                 new_state.state_graph = graph_tmp
    #                 new_state.state_room_positions = layout_tmp.get_room_positions()
    #                 new_state.state_energy = energy_tmp
    #                 new_state.move_rooms_to_scene_centre(graph_tmp)
    #                 new_state.insert_to_new_states(new_states, graph_tmp)
    #                 if new_states.size() >= NUM_SOLUTIONS_TO_TRACK:
    #                     return True
    #
    #                 if flag_last_chain and (self.solution_count + int(new_states.size()) >= CLevelConfig.TARGET_NUM_SOLUTIONS):
    #                     return True
    #
    #                 #new_states.push_back(new_state)
    #
    #             flag_accept = False
    #
    #             if energy_tmp < energy_current:
    #                 # Energy is lower, accept
    #                 flag_accept = True
    #                 if energy_tmp < energy_min:
    #                     energy_min = energy_tmp
    # #ifndef PERFORMANCE_TEST
    #                     print(f'A minimum energy: {energy_min}')
    # #endif
    # #ifdef DUMP_INTERMEDIATE_OUTPUT
    #                     layout_best = layout_tmp
    #                     graph_best = graph_tmp
    #                     for (n = 0; n < graph_best.num_nodes; n++):
    #                         pn = layout_best.get_room_positions()[n]
    #                         graph_best.get_node(n).set_pos(pn)
    #
    #                     #std.ofstream fout
    #                     ##fout.open(CLevelConfig.AddOutputPrefix('log.txt').c_str(), std.ios_base.app)
    #                     #fout << self.best_sol_count}\t{energy_min << std.endl
    #                     graph_best.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.xml', self.best_sol_count)).c_str())
    #                     open_doors(layout_best, &graph_best, True)
    #                     layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'tmpBest_%03d.svg', self.best_sol_count)).c_str())
    #                     self.best_sol_count += 1
    # #endif
    #             else:
    #                 delta_e = abs(energy_tmp - energy_current)
    #
    #                 # Energy is higher...
    #                 if i == 0 and j == 0:
    #                     delta_e_avg = delta_e
    #                     delta_e_avg *= CLevelConfig.m_delta_escaling
    #
    #                 # Generate probability of acceptance...
    #                 prob = exp(-(energy_tmp - energy_current) / (delta_e_avg * t))
    #                 r = rand() / float(RAND_MAX)
    #                 if r < prob:
    #                     flag_accept = True
    #                 else:
    #                     flag_accept = False
    #
    #             if flag_accept:
    #                 delta_e = abs(energy_tmp - energy_current)
    #                 self.layout = layout_tmp
    #                 graph = graph_tmp
    #                 energy_current = energy_tmp
    #                 if delta_e != 0.0:
    #                     na += 1
    #                     delta_e_avg = (delta_e_avg * (na - 1.0f) + delta_e) / float(na)
    #                 flag_was_accepted = True
    #
    #             pick_index_count +=1
    #             pick_index_count = pick_index_count % int(len(indices))
    #
    #         if not flag_was_accepted:
    #             num_failures += 1
    #
    #         # Lower the temperature for next cycle
    #         t *= frac
    #
    # #ifndef PERFORMANCE_TEST
    #     print(f'Final energy: {energy_min}')
    # #endif
    #     if not new_states:
    # #ifdef DUMP_INTERMEDIATE_OUTPUT
    #         print(f'Empty solution set!')
    #         graph_best.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'backTracking_level%02d_%03d.xml', self.backtrack_level, self.backtrack_count)).c_str())
    #         for (n = 0; n < graph_best.num_nodes; n++):
    #             graph_best.get_node(n).set_flag_visited(False)
    #
    #         for (n = 0; n < int(indices.size()); n++):
    #             graph_best.get_node(indices[n]).set_flag_visited(True)
    #
    #         for (n = 0; n < int(weighted_indices.size()); n++):
    #             graph_best.get_node((*weighted_indices)[n]).set_flag_visited(False)
    #
    #         layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'backTracking_level%02d_%03d.svg', self.backtrack_level, self.backtrack_count)).c_str())
    #         layout_best.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'backTrackingPartial_level%02d_%03d.svg', self.backtrack_level, self.backtrack_count)).c_str(), 400, 400, True, &graph_best)
    # #endif
    #         return False
    #
    # #ifndef PERFORMANCE_TEST
    #     print(f'Number of valid states: {len(new_states)})
    # #endif
    #     sort(new_states.begin(), new_states.end(), CompareStateEnergySmallerFirst)
    #     num_solutions_to_track = min(int(new_states.size()), NUM_SOLUTIONS_TO_TRACK)
    #     std.vector<CurrentState> newer_states(num_solutions_to_track)
    #     for (i = 0; i < num_solutions_to_track; i++):
    #         newer_states[i] = new_states[i]
    #
    #     new_states = newer_states
    #
    #     return True
    #
    def solve_1d_chainILS(self, indices, old_state, new_states):
        graph = old_state.state_graph
        self.set_sequence_as_1d_chain(indices, graph)
        new_states.clear()
        if graph.get_node(indices[0]).flag_fixed:
            old_state.insert_to_new_states(new_states, graph)
            return True

        # Borrow the parameters from simulated annealing...
        n = LevelConfig().SA_NUM_OF_CYCLES
        m = LevelConfig().SA_NUM_OF_TRIALS
        # CRoomLayout layout_best; # Current best result so far
        #collide_area = 0
        #connectivity = 00
        energy_min = 1e10
        energy_history = 1e10
        pick_index_count = 0
        for i in range(n):
            layout_history = self.layout
            graph_history = graph
            if i != 0:
                # Introduce perturbation...
                self.randomly_adjust_one_room(self.layout, graph, indices, None)

            energy_tmp, collide_area, connectivity = self.get_layout_energy(self.layout, graph)
            energy_current = energy_tmp
            if i == 0:
                energy_min = energy_current
                print(f'Initial energy: {energy_current}')

            for j in range(m):
                graph_tmp = graph
                layout_tmp = self.layout
                self.randomly_adjust_one_room(layout_tmp, graph_tmp, indices, None)
    #if 1 # New on 08/16/2013
                if self.flag_visited_no_node:
                    p_min = Vector2(1e10, 1e10)
                    p_max = Vector2(-1e10, -1e10)
                    for d in range(len(indices)):
                        idx = indices[d]
                        pj = layout_tmp.rooms[idx].get_room_centre()
                        for k in range(2):
                            p_min[k] = min(p_min[k], pj[k])
                            p_max[k] = max(p_max[k], pj[k])
                    pos_cen = (p_min + p_max) * 0.5
                    for d in range(len(indices)):
                        idx = indices[d]
                        layout_tmp.rooms[idx].translate_room(-pos_cen)
    #endif
                energy_tmp, collide_area, connectivity = self.get_layout_energy(layout_tmp, graph_tmp)
                #print(f'energy_tmp: {energy_tmp}', collide_area, connectivity)
                if collide_area <= NUMERICAL_TOLERANCE and connectivity <= NUMERICAL_TOLERANCE:
                    new_state = old_state
                    new_state.state_graph = graph
                    new_state.state_room_positions = layout_tmp.get_room_positions()
                    #print('state_room_positions:', new_state.state_room_positions)
                    new_state.state_energy = energy_tmp
                    new_state.move_rooms_to_scene_centre(graph)
                    new_state.insert_to_new_states(new_states, graph)

                if energy_tmp < energy_current:
                    if energy_tmp < energy_min:
                        # layout_best = layout_tmp
                        energy_min = energy_tmp
    #ifndef PERFORMANCE_TEST
                        print(f'A minimum energy: {energy_min}')
    #endif
                    self.layout = layout_tmp
                    graph = graph_tmp
                    energy_current = energy_tmp

                pick_index_count += 1
                pick_index_count = pick_index_count % len(indices)

            if i == 0 or energy_min < energy_history:
                energy_history = energy_min
            else:
                self.layout = layout_history
                graph = graph_history

        print(f'Final energy: {energy_min}')
        if not new_states:
            print(f'Empty solution set!')
            return False

        print(f'Number of valid states: {len(new_states)}')
        sort(new_states.begin(), new_states.end(), CompareStateEnergySmallerFirst)
        num_solutions_to_track = min(len(new_states), LevelConfig().NUM_SOLUTIONS_TO_TRACK)
        newer_states = []
        for i in range(num_solutions_to_track):
            newer_states.append(new_states[i])
        new_states[:] = newer_states

        return True

    def set_current_state(self, s):
        self.graph = s.state_graph
        self.room_positions = s.state_room_positions
        self.layout = self.get_layout(self.graph, self.room_positions)

    def set_sequence_as_1d_chain(self, indices, graph):
        self.sequence.clear()
        for i in range(len(indices)):
            idx = graph.get_node(indices[i]).type
            idx = idx % self.templates.num_templates
            self.sequence.append(idx)

    def set_visited_neighbours(self, indices):
        self.visited_neighbours.clear()
        for i in range(len(self.visited_neighbours)):
            node_idx = indices[i]
            neighbors = self.graph.get_node(node_idx).get_neighbours()
            for j in range(len(neighbors)):
                neighbour_idx = neighbors[j]
                if self.graph.get_node(neighbour_idx).flag_visited:
                    self.visited_neighbours[i].append(neighbour_idx)
    #
    # def DumpSolutionIntoXML(self):
    #     graphSol = *self.graph
    #     for (i = 0; i < graphSol.num_nodes; i++):
    #         pi = room_positions[i]
    #         graphSol.get_node(i).set_pos(pi[0], pi[1])
    #
    #     graphSol.SaveGraphAsXML(CLevelConfig.AddOutputPrefix(sprint(f'dbg_%03d.xml', self.solution_count)).c_str())
    #     layoutSol = get_layout(self.graph, room_positions)
    #     open_doors(layoutSol, self.graph)
    #     layoutSol.SaveLayoutAsSVG(CLevelConfig.AddOutputPrefix(sprint(f'dbg_%03d.svg', self.solution_count)).c_str())
    #
    # def randomly_pick_one_room(self, layout):
    #     picked_room_index = random.randint(0, layout.num_rooms - 1)
    #     return picked_room_index

    def randomly_pick_one_room(self, layout, indices=None, weighted_indices=None):
        if weighted_indices is not None:
            tmp_indices = weighted_indices
            chain_length = len(tmp_indices)
            #for (i = 0; i < indices.size(); i++):
            for i in range(len(indices)):
                energy_tmp = layout.get_room(indices[i]).energy
                if energy_tmp > 1.1:
                    tmp_indices.append(indices[i])

            #picked_room_index = int(rand() / float(RAND_MAX) * chainLength)
            #picked_room_index = picked_room_index % chainLength
            picked_room_index = random.randint(0, chain_length - 1)
            picked_room_index = tmp_indices[picked_room_index]
            return picked_room_index

        elif indices is not None:
            chain_length = len(indices)
            #picked_room_index = int(rand() / float(RAND_MAX) * chainLength)
            #picked_room_index = picked_room_index % chainLength
            picked_room_index = random.randint(0, chain_length - 1)
            picked_room_index = indices[picked_room_index]
            return picked_room_index
        else:
            picked_room_index = random.randint(0, layout.num_rooms - 1)
            return picked_room_index

    def randomly_pick_another_room(self, layout, picked_index):
        num_rooms = layout.num_rooms
        other_room_index = picked_index
        while other_room_index == picked_index:
            other_room_index = random.randint(0, num_rooms - 1)
            #other_room_index = int(rand() / float(RAND_MAX) * num_rooms)
            #other_room_index = other_room_index % num_rooms
        return other_room_index

    def get_connected_indices(self, graph, picked_index, flag_visited_only=True):
        indices = []
        for i in range(len(graph.edges)):
            edge = graph.get_edge(i)
            idx0 = edge.idx0
            idx1 = edge.idx1
            if idx0 != picked_index and idx1 != picked_index:
                continue

            idx = idx1 if idx0 == picked_index else idx0
            if not graph.get_node(idx).flag_visited and flag_visited_only:
                continue

            indices.append(idx)

        return indices

    def randomly_adjust_one_room(self, layout, graph, indices, weighted_indices):
        num_rooms = layout.num_rooms
        if num_rooms <= 1:
            return -1

        r = random.random()

#if 0 # Before 07/16/2013
        # if  r < 0.25:
        #     randomly_adjust_one_room01(layout, graph, indices)
        # 
        # elif  r < 0.5:
        #     randomly_adjust_one_room02(layout, graph, indices)
        # 
        # elif  r < 0.75 or not FLAG_ENABLE_TYPE_CHANGE:
#else:
        if r < 0.75 or not LevelConfig().FLAG_ENABLE_TYPE_CHANGE: # nv: was 0.9
 #endif
            if not LevelConfig().FLAG_RANDOM_WALK:
                return self.randomly_adjust_one_room03(layout, graph, indices, weighted_indices)
            else:
                return self.gradient_descent_one_room(layout, graph, *weighted_indices)
        else:
            return self.randomly_adjust_one_room04(layout, graph, indices, weighted_indices)

    # def randomly_adjust_one_room01(self, layout, graph, indices):
    #     picked_room_index = self.randomly_pick_one_room(layout, indices, None)
    #     picked_room = layout.get_room(picked_room_index)
    #
    #     for i in range(picked_room.num_edges):
    #         edge = picked_room.get_edge(i)
    #         pr2 = edge.GetPos2() - edge.GetPos1()
    #         pr = Vector3(pr2[0], pr2[1], 0.0)
    #         norm = Vector2(pr[1], -pr[0])
    #         norm.normalize()
    #         distMin = 1e10
    #         CRoomEdge edgeNearest
    #         for (j = 0; j < layout.Getnum_rooms(); j++):
    #             if j == picked_room_index:
    #                 continue
    #
    #             if (graph.get_node(j).flag_visited == False) continue
    #             other_room = layout.get_room(j)
    #             for (k = 0; k < other_room.GetNumOfEdges(); k++):
    #                 otherEdge = other_room.get_edge(k)
    #                 otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
    #                 otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
    #                 cp = cross(pr, otherPr)
    #                 if mag2(cp) > 0.0001f:
    #                     continue
    #
    #                 prTmp = otherEdge.GetPos1() - edge.GetPos1()
    #                 distTmp = std.abs(dot(norm, prTmp))
    #                 if distTmp < distMin:
    #                     distMin = distTmp
    #                     edgeNearest = otherEdge
    #
    #         if distMin < 0.3f:
    #             pr = edgeNearest.GetPos1() - edge.GetPos1()
    #             d = dot(norm, pr)
    #             dp = d * norm
    #             picked_room.translate_room(dp)

    # def randomly_adjust_one_room02(self, layout, graph, indices):
    #     picked_room_index = randomly_pick_one_room(layout, indices, NULL)
    #     picked_room = layout.get_room(picked_room_index)
    #
    #     numOfEdges = picked_room.GetNumOfEdges()
    #     pickedEdgeIndex = int(rand() / float(RAND_MAX) * numOfEdges)
    #     pickedEdgeIndex = pickedEdgeIndex % numOfEdges
    #     edge = picked_room.get_edge(pickedEdgeIndex)
    #     pr2 = edge.GetPos2() - edge.GetPos1()
    #     pr = v3f(pr2[0], pr2[1], 0.f)
    #     norm = v2f(pr[1], -pr[0])
    #     norm = normalize(norm)
    #
    #     other_room_index = randomly_pick_another_room(layout, picked_room_index)
    #     other_room = layout.get_room(other_room_index)
    #     distMin = 1e10
    #     CRoomEdge edgeNearest
    #
    #     for (k = 0; k < other_room.GetNumOfEdges(); k++):
    #         otherEdge = other_room.get_edge(k)
    #         otherPr2 = otherEdge.GetPos2() - otherEdge.GetPos1()
    #         otherPr = v3f(otherPr2[0], otherPr2[1], 0.f)
    #         cp = cross(pr, otherPr)
    #         if mag2(cp) > 0.0001f:
    #             continue
    #
    #         prTmp = otherEdge.GetPos1() - edge.GetPos1()
    #         distTmp = std.abs(dot(norm, prTmp))
    #         if distTmp < distMin:
    #             distMin = distTmp
    #             edgeNearest = otherEdge
    #
    #     if distMin < 0.3f:
    #         pr = edgeNearest.GetPos1() - edge.GetPos1()
    #         d = dot(norm, pr)
    #         dp = d * norm
    #         picked_room.translate_room(dp)
    #
    # def gradient_descent_one_room(self, layout, graph, indices):
    #     collide_area = 0.f
    #     connectivity = 0.f
    #     myEnergy = get_layout_energy(layout, graph, collide_area, connectivity)
    #     picked_room_index = randomly_pick_one_room(layout, indices, NULL)
    #
    #     besti = -1
    #     bestEnergy = 10e6
    #     std.vector<int> candidateAngles
    #
    #     for (i = 0; i < 360; i += 10):
    #         angle = i * atan(1.f) * 4.f / 180.0f
    #         l = layout
    #         picked_room = l.get_room(picked_room_index)
    #
    #          one_step_length = 0.1f
    #         dp = v2f(cosf(angle) * one_step_length, sinf(angle) * one_step_length)
    #
    #         picked_room.translate_room(dp)
    #         collide_area = 0.f
    #         connectivity = 0.f
    #         energy_min = get_layout_energy(l, graph, collide_area, connectivity)
    #         if energy_min < bestEnergy or besti == -1:
    #             bestEnergy = energy_min
    #             besti = i
    #
    #         if energy_min < myEnergy:
    #             candidateAngles.push_back(i)
    #
    #     #   angle = besti * M_PI / 180.0f
    #     float angle
    #
    #     if candidateAngles.empty():
    #         # Wellnot  I'm stuck on a local minimum.
    #         angle = random2(36) * 10 * atan(1.f) * 4.f / 180.0f
    #     else:
    #         angle = candidateAngles[random2(candidateAngles.size())] * atan(1.f) * 4.f / 180.0f
    #
    #     stepSize = 0.1f
    #     bestStep = -1
    #     bestEnergy = 10e6
    #
    #     for (iters = 0; iters < 4; iters++):
    #         for (stepLength = stepSize; stepLength <= 2; stepLength += stepSize):
    #             l = layout
    #             picked_room = l.get_room(picked_room_index)
    #
    #             dp = v2f(cosf(angle) * stepLength, sinf(angle) * stepLength)
    #
    #             picked_room.translate_room(dp)
    #             collide_area = 0.f
    #             connectivity = 0.f
    #             energy_min = get_layout_energy(l, graph, collide_area, connectivity)
    #             if energy_min < bestEnergy or bestStep == -1:
    #                 bestEnergy = energy_min
    #                 bestStep = stepLength
    #
    #         if myEnergy - bestEnergy < 0.00001:
    #             stepSize /= 2.0f
    #
    #     dp = v2f(cosf(angle) * bestStep, sinf(angle) * bestStep)
    #
    #     # do the actual translation on the room itself
    #     picked_room = layout.get_room(picked_room_index)
    #     picked_room.translate_room(dp)
    #     return picked_room_index
    #
    def randomly_adjust_one_room03(self, layout, graph, indices, weighted_indices):
        picked_room_index = self.randomly_pick_one_room(layout, indices, weighted_indices)
        picked_room = layout.rooms[picked_room_index]
        self.sample_config_space_for_picked_room(layout, graph, indices, picked_room_index)
        return picked_room_index

    def sample_config_space_for_picked_room(self, layout, graph, indices, picked_room_index):
        picked_room = layout.rooms[picked_room_index]
        config_space = ConfigSpace()
        connected_indices = self.get_connected_indices(graph, picked_room_index)
        if len(connected_indices) >= 1:
            random.shuffle(connected_indices)
            idx0 = connected_indices[0]
            config_space0 = ConfigSpace(layout.rooms[idx0], picked_room)
            config_space = config_space0
            for i in range(len(connected_indices)):
                config_space_tmp = ConfigSpace(layout.rooms[connected_indices[i]], picked_room)
                config_space_new = ConfigSpace.find_intersection(config_space, config_space_tmp)
                if not config_space_new.config_lines:
                    break
                else:
                    config_space = config_space_new

        while_cnt = 0
        while not config_space.config_lines:
            other_room_index = self.randomly_pick_another_room(layout, picked_room_index)
            other_room = layout.rooms[other_room_index]
            config_space = ConfigSpace(other_room, picked_room)
            while_cnt += 1
            if while_cnt >= 1000:
                print(f'Break from the while loop after reaching enough number of trials!')
                return

        pos = config_space.randomly_sample_config_space()
        dp = pos - picked_room.get_room_centre()
        picked_room.translate_room(dp)

    def randomly_adjust_one_room04(self, layout, graph, indices, weighted_indices):
        num_templates = self.templates.num_templates
        if num_templates <= 1:
            return -1

        picked_room_index = self.randomly_pick_one_room(layout, indices, weighted_indices)
        picked_room = layout.rooms[picked_room_index]

        type_old = graph.get_node(picked_room_index).type
        type_new = type_old
        boundary_old = graph.get_node(picked_room_index).boundary_type
        boundary_new = -1
        while_cnt = 0
        while (type_new == type_old or boundary_new != boundary_old or self.templates.rooms[type_new].boundary_type == 1):
            #type_new = int(rand() / float(RAND_MAX) * num_templates)
            #type_new = type_new % num_templates
            type_new = random.randint(0, num_templates - 1)
            boundary_new = self.templates.rooms[type_new].boundary_type
            while_cnt += 1
            if while_cnt >= 1000:
                print(f'Break from the while loop after reaching enough number of trials in randomly_adjust_one_room04()!')
                return -1


        graph.get_node(picked_room_index).type = type_new
        room = copy.deepcopy(self.templates.rooms[type_new])
        p1 = room.get_room_centre()
        p2 = picked_room.get_room_centre()
        dp = p2 - p1
        room.translate_room(dp)
        #picked_room = room
        layout.rooms[picked_room_index] = room
#if 1 # New on 09/15/2013
        #sample_config_space_for_picked_room(layout, graph, indices, picked_room_index)
#endif
        return picked_room_index

    # def get_layout_energy_early_out(self, layout, graph, collide_area, connectivity, room_moved, energy_tmp, energy_current):
    #     layout.reset_room_energies()
    #     energy_tmp = 1.0
    #
    #     # do connectivity first, it's (probably?) cheaper
    #
    #     if SIGMA_CONNECTIVITY > 0:
    #         connectivity = self.check_room_connectivity(layout, graph, True, room_moved)
    #         energy_tmp *= math.exp(connectivity * SIGMA_CONNECTIVITY)
    #
    #     if energy_tmp > energy_current:
    #         return False
    #
    #     if SIGMA_COLLIDE > 0:
    #         collide_area = self.layout_collide(layout, graph, True, room_moved)
    #         (*energy_tmp) *= math.exp(collide_area * SIGMA_COLLIDE)
    #
    #     if energy_tmp > energy_current:
    #         return False
    #
    #     if SIGMA_CONTACT > 0:
    #         contact_area = self.layout_contact(layout, graph, True, FLAG_NON_OVERLAP_CONTACT)
    #         energy_tmp *= math.exp(-contact_area * SIGMA_CONTACT)
    #
    #     return True
    #
    def get_layout_energy(self, layout, graph, room_moved=-1, do_contact=False, indices=None):
        layout.reset_room_energies()
        layout_energy = 1.0
        if LevelConfig().SIGMA_COLLIDE > 0:
            collide_area = self.layout_collide(layout, graph, True, room_moved)
            layout_energy *= math.exp(collide_area * LevelConfig().SIGMA_COLLIDE)

        if LevelConfig().SIGMA_CONNECTIVITY > 0:
            connectivity = self.check_room_connectivity(layout, graph, True, room_moved)
            layout_energy *= math.exp(connectivity * LevelConfig().SIGMA_CONNECTIVITY)

        # if LevelConfig().SIGMA_CONTACT > 0 and do_contact:
        #     contact_area = -self.layout_contact(layout, graph, True, LevelConfig().FLAG_NON_OVERLAP_CONTACT, indices)
        #     if contact_area >= 0.0:
        #         contact_area = 0.0
        #     if contact_area < 0:
        #         layout_energy *= math.exp(contact_area / LevelConfig().SIGMA_CONTACT)

        return layout_energy, collide_area, connectivity

    def check_room_connectivity(self, layout, graph, flag_visited_only=False, room_moved=-1):
        connectivity = 0.0
        if graph is None:
            return connectivity

        for i in range(len(graph.edges)):
            edge = graph.get_edge(i)
            idx0 = edge.idx0
            idx1 = edge.idx1
            flag_visited0 = graph.get_node(idx0).flag_visited
            flag_visited1 = graph.get_node(idx1).flag_visited
            if flag_visited_only and (not flag_visited0 or not flag_visited1):
                continue

            flag_fixed0 = graph.get_node(idx0).flag_fixed
            flag_fixed1 = graph.get_node(idx1).flag_fixed
            if flag_fixed0 and flag_fixed1:
                continue

            if room_moved == -1 or room_moved == idx0 or room_moved == idx1 or layout.cached_connectivities.find((idx0, idx1)) == layout.cached_connectivities.end():
                contact_area = room_contact(layout.rooms[idx0], layout.rooms[idx1])
                if contact_area <= LevelConfig().ROOM_CONTACT_THRESHOLD:
                    if LevelConfig().FLAG_DISCRETE_CONNECT_FUNC:
                        connectivity += 1
                        layout.cached_connectivities[(idx0, idx1)] = 1
                    else:
                        d = room_distance(layout.rooms[idx0], layout.rooms[idx1])
                        d += LevelConfig().ROOM_CONTACT_THRESHOLD
                        layout.cached_connectivities[(idx0, idx1)] = d
                        connectivity += d
                    factor = 1.1
                    layout.get_room(idx0).update_energy(factor)
                    layout.get_room(idx1).update_energy(factor)
                else:
                    layout.cached_connectivities[(idx0, idx1)] = 0.0
            else:
                connectivity += layout.cached_connectivities[(idx0, idx1)]
        return connectivity
    
    def layout_collide(self, layout, graph, flag_visited_only=False, room_that_moved=-1):
        collide_area_total = 0
        collide_count = 0
        num_rooms = layout.num_rooms
        for i in range(num_rooms):
            for j in range(num_rooms):
                flag_visited0 = graph.get_node(i).flag_visited
                flag_visited1 = graph.get_node(j).flag_visited
                if flag_visited_only and (not flag_visited0 or not flag_visited1):
                    continue

                flag_fixed0 = graph.get_node(i).flag_fixed
                flag_fixed1 = graph.get_node(j).flag_fixed
                if flag_fixed0 and flag_fixed1:
                    continue

                if room_that_moved == -1 or room_that_moved == i or room_that_moved == j or layout.cached_collision_energies.find((i, j)) == layout.cached_collision_energies.end():
                    collide_area = self.room_collides(layout.rooms[i], layout.rooms[j])
                    if collide_area > 0:
                        collide_area_total += collide_area
                        collide_count += 1
                        factor = math.exp(collide_area)
                        layout.rooms[i].update_energy(factor)
                        layout.rooms[j].update_energy(factor)
                        layout.cached_collision_energies[(i, j)] = collide_area
                    else:
                        layout.cached_collision_energies[(i, j)] = collide_area
                else:
                    collide_area_total += layout.cached_collision_energies[(i, j)]

    #ifdef PRINT_OUT_DEBUG_INFO
        print(f'Number of colliding room pairs: {collide_count}')
        print(f'Total area of colliding area: {collide_area_total}')
    #endif
        return collide_area_total

    # def layout_collide(self, layout):
    #     collide_area_total = 0
    #     collide_count = 0
    #     num_rooms = layout.num_rooms
    #     for i in range(num_rooms):
    #         for j in range(num_rooms):
    #             if layout.get_room(i).boundary_type == 1 and layout.get_room(j).boundary_type == 1:
    #                 continue
    #
    #             collide_area = self.room_collides(layout.get_room(i), layout.get_room(j))
    #             if collide_area > 0:
    #                 collide_area_total += collide_area
    #                 collide_count += 1
    #
    # #ifdef PRINT_OUT_DEBUG_INFO
    #     print(f'Number of colliding room pairs: {collide_count}')
    #     print(f'Total area of colliding area: {collide_area_total}')
    # #endif
    #     return collide_area_total
    #
    def room_collides(self, room1, room2):
        collide_area = -1

        # Test the bounding box first...
        bb1 = AABB2(*room1.get_room_bounding_box())
        bb2 = AABB2(*room2.get_room_bounding_box())
        if not self.test_bounding_box_collides(bb1, bb2):
            return 0.0

        # Use the Clipper library...
        collide_area = compute_collide_area(room1, room2)

        return collide_area

    # def BoundingBoxCollidesArea(self, bb1, bb2):
    #     collide_area = -1.f
    #     for (j = 0; j < 2; j++):
    #         if bb1.m_posMax[j] < bb2.m_posMin[j] or bb1.m_posMin[j] > bb2.m_posMax[j]:
    #             return collide_area
    #
    #     collide_area = 1.f
    #     for (j = 0; j < 2; j++):
    #         p_min = max(bb1.m_posMin[j], bb2.m_posMin[j])
    #         p_max = min(bb1.m_posMax[j], bb2.m_posMax[j])
    #         if (p_min > p_max) return -1.f
    #         pd = p_max - p_min
    #         collide_area *= pd
    #
    #     return collide_area
    #
    def test_bounding_box_collides(self, bb1, bb2):
        for j in range(2):
            if bb1.pos_max[j] < bb2.pos_min[j] or bb1.pos_min[j] > bb2.pos_max[j]:
                return False
        return True

    # def layout_contact(self, layout, graph, flag_visited_only ''' = False ''', flagNonOverlap ''' = False ''', indices, room_that_moved ''' probably == null '''):
    #     contact_areaTotal = 0.f
    #     contactCount = 0
    #     num_rooms = layout.Getnum_rooms()
    #
    #     for (i = 0; i < num_rooms; i++):
    #         flag_visited0 = graph.get_node(i).flag_visited
    #         if flag_visited0 == False:
    #             continue
    #
    #         badNeighbour = False
    #         std.vector<int>neighbours = graph.get_node(i).GetNeighbors()
    #         for (j = 0; j < neighbours.size(); j++):
    #             found = False
    #             for (k = 0; k < indices.size(); k++):
    #                 if (*indices)[k] == neighbours[j]:
    #                     found = True
    #                     break
    #
    #             if not found:
    #                 badNeighbour = True
    #                 break
    #
    #         if not badNeighbour:
    #             continue
    #
    #         perimeter = RoomPerimeter(layout.get_room(i))
    #
    #         for (j = i + 1; j < num_rooms; j++):
    #             flag_visited1 = graph.get_node(j).flag_visited
    #             if flag_visited_only and (flag_visited0 == False or flag_visited1 == False):
    #                 continue
    #
    #             if i == room_that_moved or j == room_that_moved or room_that_moved == -1 or layout.cachedContacts.find((i, j)) == layout.cachedContacts.end():
    #                 if room_collides(layout.get_room(i), layout.get_room(j)) > 0.f:
    #                     layout.cachedContacts[(i, j)] = 0.0f
    #                     continue
    #
    #                 contact_area = room_contact(layout.get_room(i), layout.get_room(j))
    #                 if (contact_area > ROOM_CONTACT_THRESH) #0.f
    #                     contact_area -= ROOM_CONTACT_THRESH
    #                     layout.cachedContacts[(i, j)] = contact_area
    #                     perimeter -= contact_area
    #                     contactCount++
    #
    #             else:
    #                 perimeter -= layout.cachedContacts[(i, j)]
    #
    #         if perimeter > 0:
    #             contact_areaTotal += perimeter
    #
    # #ifdef PRINT_OUT_DEBUG_INFO
    #     print(f'Number of contacting room pairs: {contactCount}')
    #     print(f'Total area of contacting area: {contact_areaTotal}')
    # #endif
    #     return contact_areaTotal
    #
    #
    # # def ComputeLabelPosition(self, idx, graph, labelRad):
    # #     Vector2p_min, p_max
    # #     graph.GetGraphBoundingBox(p_min, p_max)
    # #     vMax = max(max(std.abs(p_min[0]), std.abs(p_min[1])), max(std.abs(p_max[0]), std.abs(p_max[1])))
    # #      pos = graph.get_node_pos(idx)
    # #      rad = labelRad
    # #     n = 32
    # #     dMinMax = -1e10
    # #     Vector2piMax
    # #     for (i = 0; i < n; i++):
    # #         angle = atan(1.f) * 8.f * float(i) / float(n)
    # #         cv = -cos(angle) * rad
    # #         sv = sin(angle) * rad
    # #         pi = pos + v2f(cv, sv)
    # #         if pi[0] < -vMax or pi[0] > vMax or pi[1] < -vMax or pi[1] > vMax:
    # #             continue
    # #
    # #         dMin = 1e10
    # #         for (j = 0; j < graph.GetNumOfEdges(); j++):
    # #             edge = graph.get_edge(j)
    # #             idx1 = edge.idx0
    # #             idx2 = edge.idx1
    # #             if idx1 != idx and idx2 != idx:
    # #                 continue
    # #
    # #             pos1 = graph.get_node_pos(idx1)
    # #             pos2 = graph.get_node_pos(idx2)
    # #             dTmp = PointToSegmentSqDistance(pi, CLineBase(pos1, pos2))
    # #             if dTmp < dMin:
    # #                 dMin = dTmp
    # #
    # #
    # #         if dMin >= rad * rad:
    # #             return pi
    # #
    # #         if dMin > dMinMax:
    # #             dMinMax = dMin
    # #             piMax = pi
    # #
    # #
    # #     return piMax
    # #
