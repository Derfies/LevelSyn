#include "RoomTemplates.h"

#include <sstream>

def PrintTemplates(self):
    std.cout << "There are " << m_rooms.size() << " room templatesnot \n"
    for (i = 0; i < int(m_rooms.size()); i++)
        std.vector<v2f> vertices = m_rooms[i].GetVertices()
        std.cout << "Room " << i << " with " << vertices.size() << " vertices:\n"
        for (j = 0; j < int(vertices.size()); j++)
            std.cout << vertices[j] << " "

        std.cout << std.endl



def LoadTemplatesFromXML(self, fileName):
    # Clear the current templates...
    ClearTemplates()

    tinyxml2.XMLDocument doc
    if doc.LoadFile(fileName) != tinyxml2.XML_SUCCESS:
        std.cout << "Failed to load templates from " << fileName << "not \n"
        return False

    #doc.Print()

    xmlRoot = doc.RootElement()
    assert(xmlRoot)

    xmlNode = xmlRoot.FirstChild()
    while (xmlNode != 0)
        if strcmp(xmlNode.Value(), "Room") == 0:
            std.vector<int> doorPositions
            # Parse a room...
            CRoom room
            std.vector<v2f> vertices
            xmlChildNode = xmlNode.FirstChild()
            while (xmlChildNode != 0)
                if strcmp(xmlChildNode.Value(), "Vertex") == 0:
                    # Parse a vertex...
                    float px, py
                    rx = xmlChildNode.ToElement().QueryFloatAttribute("px", &px)
                    ry = xmlChildNode.ToElement().QueryFloatAttribute("py", &py)
                    if rx == tinyxml2.XML_SUCCESS and ry == tinyxml2.XML_SUCCESS:
                        v2f pos
                        pos[0] = px
                        pos[1] = py
                        vertices.push_back(pos)


                elif strcmp(xmlChildNode.Value(), "Shift") == 0:
                    float px, py
                    rx = xmlChildNode.ToElement().QueryFloatAttribute("px", &px)
                    ry = xmlChildNode.ToElement().QueryFloatAttribute("py", &py)
                    if rx == tinyxml2.XML_SUCCESS and ry == tinyxml2.XML_SUCCESS:
                        v2f pos
                        pos[0] = px
                        pos[1] = py
                        room.SetCenterShift(pos)


                elif strcmp(xmlChildNode.Value(), "Boundary") == 0:
                    int type
                    r = xmlChildNode.ToElement().QueryIntAttribute("type", &type)
                    if r == tinyxml2.XML_SUCCESS:
                        room.SetBoundaryType(type)


                elif strcmp(xmlChildNode.Value(), "Door") == 0:
                    int idx
                    r = xmlChildNode.ToElement().QueryIntAttribute("edgeIndex", &idx)
                    if r == tinyxml2.XML_SUCCESS:
                        doorPositions.push_back(idx)


                xmlChildNode = xmlChildNode.NextSibling()

            room.SetVertices(vertices)
            if doorPositions.empty() == False:
                room.ResetDoorFlags()
                for (i = 0; i < int(doorPositions.size()); i++)
                    room.SetDoorFlag(doorPositions[i], True)


            AddTemplate(room)


        # Move to the next sibling...
        xmlNode = xmlNode.NextSibling()


    SetRoomTypes()
    #PrintTemplates()
    return True


def SaveTemplatesAsXML(self, fileName):
     str = "\t<?xml version=\"1.0\" standalone=\"yes\" ?>\n"
                      "<not -- room template data -.\n"
                      "<Templates>\n"
                      "</Templates>\n"
    tinyxml2.XMLDocument doc
    doc.Parse(str)
    root = doc.RootElement()
    # Dump nodes...
    for (i = 0; i < GetNumOfTemplates(); i++)
        roomElement = doc.NewElement("Room")
        for (j = 0; j < GetRoom(i).GetNumOfVertices(); j++)
            vertexElement = doc.NewElement("Vertex")
            pj = GetRoom(i).GetVertex(j)
            std.ostringstream oss0
            std.ostringstream oss1
            oss0 << pj[0]
            oss1 << pj[1]
            vertexElement.SetAttribute("px", oss0.str().c_str())
            vertexElement.SetAttribute("py", oss1.str().c_str())
            roomElement.InsertEndChild(vertexElement)

        # Dump center shift...
        shiftElement = doc.NewElement("Shift")
        shift = GetRoom(i).GetCenterShift()
        std.ostringstream oss0
        std.ostringstream oss1
        oss0 << shift[0]
        oss1 << shift[1]
        shiftElement.SetAttribute("px", oss0.str().c_str())
        shiftElement.SetAttribute("py", oss1.str().c_str())
        roomElement.InsertEndChild(shiftElement)
        # Dump boundary type...
        if GetRoom(i).GetBoundaryType() != 0:
            boundaryElement = doc.NewElement("Boundary")
            boundaryElement.SetAttribute("type", GetRoom(i).GetBoundaryType())
            roomElement.InsertEndChild(boundaryElement)

        # Dump door positions...
        if GetRoom(i).HasRestrictedDoorPosition() == True:
            std.vector<bool> doorFlags = GetRoom(i).GetDoorFlags()
            for (j = 0; j < int(doorFlags.size()); j++)
                if doorFlags[j] == False:
                    continue

                doorElement = doc.NewElement("Door")
                doorElement.SetAttribute("edgeIndex", j)
                roomElement.InsertEndChild(doorElement)


        # Add room...
        root.InsertEndChild(roomElement)


    saveFlag = doc.SaveFile(fileName)
    return saveFlag


def EnrichByRotating180Degrees(self):
    numOfTemplatesOld = GetNumOfTemplates()
    for (i = 0; i < numOfTemplatesOld; i++)
        roomNew = GetRoom(i)
        roomNew.RotateRoom(atan(1.f) * 4.f)
        AddTemplate(roomNew)

    SetRoomTypes()


def EnrichByIntroducingSizeVariation(self):
    numOfTemplatesOld = GetNumOfTemplates()
    for (n = 0; n < 3; n++)
        for (i = 0; i < numOfTemplatesOld; i++)
            roomNew = GetRoom(i)
            rx = 1.f + rand() / float(RAND_MAX)
            ry = 1.f + rand() / float(RAND_MAX)
            roomNew.ScaleRoom(v2f(rx, ry))
            AddTemplate(roomNew)


    SetRoomTypes()


def SetRoomTypes(self):
    for (i = 0; i < GetNumOfTemplates(); i++)
        m_rooms[i].SetTemplateType(i)


