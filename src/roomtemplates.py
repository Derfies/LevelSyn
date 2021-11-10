import xml.etree.cElementTree as et

from reactor.geometry.vector import Vector2
from room import Room


class RoomTemplates:
    
    def __init__(self):
        self.rooms = []

    @property
    def num_templates(self):
        return len(self.rooms)
    
    def __str__(self):
        str_ = f'There are {len(self.rooms)} room templates\n'
        for i, room in enumerate(self.rooms):
            str_ += f'Room {i} with {len(room.vertices)} vertices:\n'
            for j, vertex in enumerate(room.vertices):
                str_ += f'    {j}th vertex: {vertex}\n'
        return str_
    
    def load(self, file_path):
        self.rooms.clear()
        tree = et.parse(file_path)
        xml_root = tree.getroot()
        for xml_node in xml_root.findall('Room'):
            vertices = []
            centre_shift = None
            boundary_type = None
            door_positions = []
            for xml_child_node in xml_node:
                if xml_child_node.tag == 'Vertex':
                    px = xml_child_node.get('px')
                    py = xml_child_node.get('py')
                    pos = Vector2(float(px), float(py))
                    vertices.append(pos)
                elif xml_child_node.tag == 'Shift':
                    px = xml_child_node.get('px')
                    py = xml_child_node.get('py')
                    pos = Vector2(float(px), float(py))
                    centre_shift = pos
                elif xml_child_node.tag == 'Boundary':
                    btype = int(xml_child_node.get('type'))
                    boundary_type = btype
                elif xml_child_node.tag == 'Door':
                    idx = int(xml_child_node.get('edgeIndex'))
                    door_positions.append(idx)

            room = Room.from_positions(vertices)
            room.centre_shift = centre_shift
            room.boundary_type = boundary_type
            room.door_positions = door_positions
            room.reset_door_flags()
            # if door_positions:
            #     for door_position in door_positions:
            #         room.set_door_flag(door_position, True)

            # HAXXOR set all edges as having doors for the moment.
            #for i in range(len(room.vertices)):
            #    room.set_door_flag(i, True)
            for edge in room.get_edges():
                edge.door_flag = True

            self.rooms.append(room)

        self.set_room_types()

        return True
    
    # def SaveTemplatesAsXML(self, file_path):
    #      str = '\t<?xml version=\'1.0\' standalone=\'yes\' ?>\n'
    #                       '<not -- room template data -.\n'
    #                       '<Templates>\n'
    #                       '</Templates>\n'
    #     tinyxml2.XMLDocument doc
    #     doc.Parse(str)
    #     root = doc.RootElement()
    #     # Dump nodes...
    #     for (i = 0; i < GetNumOfTemplates(); i++)
    #         roomElement = doc.NewElement('Room')
    #         for (j = 0; j < GetRoom(i).GetNumOfVertices(); j++)
    #             vertexElement = doc.NewElement('Vertex')
    #             pj = GetRoom(i).GetVertex(j)
    #             std.ostringstream oss0
    #             std.ostringstream oss1
    #             oss0 << pj[0]
    #             oss1 << pj[1]
    #             vertexElement.SetAttribute('px', oss0.str().c_str())
    #             vertexElement.SetAttribute('py', oss1.str().c_str())
    #             roomElement.InsertEndChild(vertexElement)
    # 
    #         # Dump center shift...
    #         shiftElement = doc.NewElement('Shift')
    #         shift = GetRoom(i).GetCenterShift()
    #         std.ostringstream oss0
    #         std.ostringstream oss1
    #         oss0 << shift[0]
    #         oss1 << shift[1]
    #         shiftElement.SetAttribute('px', oss0.str().c_str())
    #         shiftElement.SetAttribute('py', oss1.str().c_str())
    #         roomElement.InsertEndChild(shiftElement)
    #         # Dump boundary type...
    #         if GetRoom(i).GetBoundaryType() != 0:
    #             boundaryElement = doc.NewElement('Boundary')
    #             boundaryElement.SetAttribute('type', GetRoom(i).GetBoundaryType())
    #             roomElement.InsertEndChild(boundaryElement)
    # 
    #         # Dump door positions...
    #         if GetRoom(i).HasRestrictedDoorPosition() == True:
    #             std.vector<bool> doorFlags = GetRoom(i).GetDoorFlags()
    #             for (j = 0; j < int(doorFlags.size()); j++)
    #                 if doorFlags[j] == False:
    #                     continue
    # 
    #                 doorElement = doc.NewElement('Door')
    #                 doorElement.SetAttribute('edgeIndex', j)
    #                 roomElement.InsertEndChild(doorElement)
    # 
    #         # Add room...
    #         root.InsertEndChild(roomElement)
    # 
    #     saveFlag = doc.SaveFile(file_path)
    #     return saveFlag
    # 
    # def EnrichByRotating180Degrees(self):
    #     numOfTemplatesOld = GetNumOfTemplates()
    #     for (i = 0; i < numOfTemplatesOld; i++)
    #         roomNew = GetRoom(i)
    #         roomNew.RotateRoom(atan(1.f) * 4.f)
    #         AddTemplate(roomNew)
    # 
    #     set_room_types()
    
    # def EnrichByIntroducingSizeVariation(self):
    #     numOfTemplatesOld = GetNumOfTemplates()
    #     for (n = 0; n < 3; n++)
    #         for (i = 0; i < numOfTemplatesOld; i++)
    #             roomNew = GetRoom(i)
    #             rx = 1.f + rand() / float(RAND_MAX)
    #             ry = 1.f + rand() / float(RAND_MAX)
    #             roomNew.ScaleRoom(v2f(rx, ry))
    #             AddTemplate(roomNew)
    # 
    # 
    #     set_room_types()
    
    def set_room_types(self):
        for i in range(self.num_templates):
            self.rooms[i].template_type = i

