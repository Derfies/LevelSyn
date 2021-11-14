import xml.etree.cElementTree as et

from reactor.geometry.vector import Vector2
from room import Room
from reactor.geometry.orthogonalpolygon import OrthogonalPolygon


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

            poly = OrthogonalPolygon.from_positions(vertices)
            room = Room(poly)#.from_positions(vertices)
            #room.centre_shift = centre_shift
            #room.boundary_type = boundary_type
            #room.door_positions = door_positions
            #room.reset_door_flags()
            # if door_positions:
            #     for door_position in door_positions:
            #         room.set_door_flag(door_position, True)

            # HAXXOR set all edges as having doors for the moment.
            #for i in range(len(room.vertices)):
            #    room.set_door_flag(i, True)
            for edge in room.get_edges():
                edge.door_flag = True

            #print(list(node.position for node in room.get_nodes()))

            self.rooms.append(room)

        self.set_room_types()

        return True
    
    def set_room_types(self):
        for i in range(self.num_templates):
            self.rooms[i].type = i
