from reactor.geometry.vector import Vector2


class RoomLayout:
    
    def __init__(self):
        self.rooms = []
        self.corridor_walls = []

    def __str__(self):
        str_ = f'Layout with {self.num_rooms} rooms\n'
        for i, room in enumerate(self.rooms):
            str_ += f'    {i}th room: {room}\n'
        return str_

    @property
    def num_rooms(self):
        return len(self.rooms)

    @property
    def num_verticies(self):
        num_vertices = 0
        for room in self.rooms:
            num_vertices += room.num_verticies
        return num_vertices

    @property
    def num_edges(self):
        num_edges = 0
        for room in self.rooms:
            num_edges += room.num_edges
        return num_edges

    @property
    def num_corridor_walls(self):
        return len(self.corridor_walls)

    def clear_layout(self):
        self.rooms.clear()

    # def GetNearestEdgePair(self, roomIdx0, roomIdx1):
    #     v2i pairMin(-1)
    #     distMin = 1e10
    #     room0 = GetRoom(roomIdx0)
    #     room1 = GetRoom(roomIdx1)
    #     numOfEdges0 = room0.GetNumOfEdges()
    #     numOfEdges1 = room1.GetNumOfEdges()
    #     for (i = 0; i < numOfEdges0; i++)
    #         idx00 = i
    #         idx01 = (i + 1) % numOfEdges0
    #         e0 = room0.GetVertex(idx01) - room0.GetVertex(idx00)
    #         p0 = (room0.GetVertex(idx01) + room0.GetVertex(idx00)) * 0.5f
    #         for (j = 0; j < numOfEdges1; j++)
    #             idx10 = j
    #             idx11 = (j + 1) % numOfEdges1
    #             e1 = room1.GetVertex(idx11) - room1.GetVertex(idx10)
    #             p1 = (room1.GetVertex(idx11) + room1.GetVertex(idx10)) * 0.5f
    #             pr = p1 - p0
    #             if std.abs(dot(e0, e1)) < 1e-10:
    #                 continue
    #
    #             distTmp = mag2(pr)
    #             if distTmp < distMin:
    #                 distMin = distTmp
    #                 pairMin[0] = i
    #                 pairMin[1] = j
    #
    #     return pairMin
    #
    def get_layout_bounding_box(self):
        pos_min, pos_max = Vector2(1e10, 1e10), Vector2(-1e10, -1e10)
        for i in range(self.num_rooms):
            pmin_tmp, pmax_tmp = self.rooms[i].get_room_bounding_box()
            for j in range(2):
                pos_min[j] = min(pos_min[j], pmin_tmp[j])
                pos_max[j] = max(pos_max[j], pmax_tmp[j])
        return pos_min, pos_max

    def move_to_scene_centre(self):
        pos_min, pos_max = self.get_layout_bounding_box()
        pos_cen = (pos_min + pos_max) * 0.5
        for i in range(self.num_rooms):
            self.rooms[i].translate_room(-pos_cen)

    def get_room_positions(self):
        room_positions = []
        for i in range(self.num_rooms):
            room_positions.append(self.rooms[i].get_room_centre())
        return room_positions

    def reset_room_energies(self):
        for room in self.rooms:
            room.reset_energy()

    # def ConvertPos(self, p, pMin, pMax, sz):
    #     pd = (p - pMin) / (pMax - pMin) * sz
    #     return int(pd)
    #
    # def ConvertPosX(self, p, pMin, pMax, sz):
    #     return ConvertPos(p, pMin, pMax, sz)
    #
    # def ConvertPosY(self, p, pMin, pMax, sz):
    #     return sz - 1 - ConvertPos(p, pMin, pMax, sz)
