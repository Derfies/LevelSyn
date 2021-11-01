

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

    def reset_room_energies(self):
        for room in self.rooms:
            room.reset_energy()

    def ConvertPos(self, p, pMin, pMax, sz):
        pd = (p - pMin) / (pMax - pMin) * sz
        return int(pd)

    def ConvertPosX(self, p, pMin, pMax, sz):
        return ConvertPos(p, pMin, pMax, sz)

    def ConvertPosY(self, p, pMin, pMax, sz):
        return sz - 1 - ConvertPos(p, pMin, pMax, sz)
