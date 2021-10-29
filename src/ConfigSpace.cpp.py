import random


class ConfigLine:

    def __init__(self, pos1, pos2):
        self.pos1 = pos1
        self.pos2 = pos2
    
    def __str__(self):
        return f'p1: {self.pos1} p2: {self.pos2}'
    
    def randomly_sample_configLine(self):

        # TODO: polish this bit.
        wt1 = random.random() / float(RAND_MAX)
        wt2 = 1
        f - wt1
        pos = wt1 * self.pos1 + wt2 * self.pos2
        return pos
    
    def randomly_sample_config_line_discrete(self):
        r = random.random()
        pos = self.pos1 if r >= 0.5 else self.pos2
        return pos
    
    def get_config_line_length(self):
        return (self.pos1 - self.pos2).mag()
    
    def get_config_line_sq_length(self):
        return (self.pos1 - self.pos2).mag2()
    
    def translate_config_line(self, trans):
        self.pos1 += trans
        self.pos2 += trans


class ConfigSpace:

    def __init__(self, room1, room2):
        type1 = room1.GetTemplateType()
        type2 = room2.GetTemplateType()
        if self.flag_precomputed and type1 >= 0 and type2 >= 0:
            cs = self.precomputed_table[type1][type2]
            cs.translate_config_space(room1.get_room_center())
            *self = cs
            return

        #cout << "Construct config space on the fly for room pair (" << type1 << ", " << type2 << ")...\n"
        CClipperWrapper wrapper
         contactThresh = CLevelConfig.m_roomContactThresh * 0.5f; # Chongyang: why self 0.5 makes everything better?
        for (i = 0; i < room1.GetNumOfEdges(); i++)
            edge1 = room1.GetEdge(i)
            for (j = 0; j < room2.GetNumOfEdges(); j++)
                edge2 = room2.GetEdge(j)
                if edge1.GetDoorFlag() == False or edge2.GetDoorFlag() == False:
                    continue

                cp = cross(edge1.GetDirection3D(), edge2.GetDirection3D())
                if mag2(cp) > g_numericalTolerance:
                    continue

                std.vector<v2f> vecPr(4)
                vecPr[0] = edge1.GetPos1() - edge2.GetPos1()
                vecPr[1] = edge1.GetPos1() - edge2.GetPos2()
                vecPr[2] = edge1.GetPos2() - edge2.GetPos1()
                vecPr[3] = edge1.GetPos2() - edge2.GetPos2()
    #ifdef ACCURATE_CONFIG_SPACE
                dir = edge1.GetDirection()
                dir = normalize(dir)
                shift = dir * contactThresh
                for (k = 0; k < 4; k++)
                    vecPr.push_back(vecPr[k] + shift)
                    vecPr.push_back(vecPr[k] - shift)

    #endif
                SortVecPr(vecPr)
                for (k = 1; k < int(vecPr.size()); k++)
                    pr1 = vecPr[k]
                    pr2 = vecPr[k - 1]
                    if mag2(pr2 - pr1) < g_numericalTolerance:
                        continue

                    pr3 = (pr1 + pr2) * 0.5f
                    room2n1 = room2
                    room2n1.TranslateRoom(pr1)
                    if wrapper.ComputeCollideArea(room1, room2n1) > g_numericalTolerance:
                        continue

    #ifdef ACCURATE_CONFIG_SPACE
                    if RoomContact(room1, room2n1) < contactThresh - g_numericalTolerance:
                        continue

    #endif
                    room2n2 = room2
                    room2n2.TranslateRoom(pr2)
                    if wrapper.ComputeCollideArea(room1, room2n2) > g_numericalTolerance:
                        continue

    #ifdef ACCURATE_CONFIG_SPACE
                    if RoomContact(room1, room2n2) < contactThresh - g_numericalTolerance:
                        continue

    #endif
                    room2n3 = room2
                    room2n3.TranslateRoom(pr3)
                    if wrapper.ComputeCollideArea(room1, room2n3) > g_numericalTolerance:
                        continue

    #ifdef ACCURATE_CONFIG_SPACE
                    if RoomContact(room1, room2n3) < contactThresh - g_numericalTolerance:
                        continue

    #endif
                    pos1 = room2.get_room_center() + pr1
                    pos2 = room2.get_room_center() + pr2
                    CConfigLine line(pos1, pos2)
                    AddConfigLine(line)



        SelfMerge()


    CConfigSpace.CConfigSpace( std.vector<CConfigLine>& vecConfigLines) : m_vecConfigLine(vecConfigLines)
        SelfMerge()

    std.vector<std.vector<CConfigSpace>> CConfigSpace.self.precomputed_table
    bool CConfigSpace.self.flag_precomputed = False

    #define ACCURATE_CONFIG_SPACE # New on 09/20/2013

    def RandomlySampleConfigSpace(self):
        r = random.random() / float(RAND_MAX)
        pos = (r >= 0.5f) ? RandomlySampleConfigSpaceContinuous() : RandomlySampleConfigSpaceDiscrete()
        return pos

    def RandomlySampleConfigSpaceContinuous(self):
        numOfLines = GetNumOfLines()
        lineIndex = int(random.random() / float(RAND_MAX) * numOfLines)
        lineIndex = lineIndex % numOfLines
        pos = m_vecConfigLine[lineIndex].randomly_sample_configLine()
        return pos

    def RandomlySampleConfigSpaceDiscrete(self):
        numOfLines = GetNumOfLines()
        lineIndex = int(random.random() / float(RAND_MAX) * numOfLines)
        lineIndex = lineIndex % numOfLines
        pos = m_vecConfigLine[lineIndex].randomly_sample_config_line_discrete()
        return pos

    def SmartlySampleConfigSpace(self):
        numOfLines = GetNumOfLines()
        std.vector<v2f> vecPos(numOfLines)
        for (i = 0; i < numOfLines; i++)
            configLine = GetConfigLine(i)
            r = random.random() / float(RAND_MAX)
            pos = (r >= 0.5f) ? configLine.randomly_sample_configLine() : configLine.randomly_sample_config_line_discrete()
            vecPos[i] = pos

        return vecPos

    def FindIntersection(self, configSpace1, configSpace2):
        CConfigSpace intersectSpace
        for (i1 = 0; i1 < configSpace1.GetNumOfLines(); i1++)
            configLine1 = configSpace1.GetConfigLine(i1)
            for (i2 = 0; i2 < configSpace2.GetNumOfLines(); i2++)
                configLine2 = configSpace2.GetConfigLine(i2)
                if configLine1.get_config_line_sq_length() < g_numericalToleranceSq and configLine2.get_config_line_sq_length() < g_numericalToleranceSq:
                    if mag2(configLine1.GetPos1() - configLine2.GetPos1()) < g_numericalToleranceSq:
                        intersectSpace.AddConfigLine(configLine1)

                    continue

                elif configLine1.get_config_line_sq_length() < g_numericalToleranceSq:
                    CRoomEdge edge(configLine2.GetPos1(), configLine2.GetPos2())
                    if PointToSegmentSqDistance(configLine1.GetPos1(), edge) < g_numericalToleranceSq:
                        intersectSpace.AddConfigLine(configLine1)

                    continue

                elif configLine2.get_config_line_sq_length() < g_numericalToleranceSq:
                    CRoomEdge edge(configLine1.GetPos1(), configLine1.GetPos2())
                    if PointToSegmentSqDistance(configLine2.GetPos1(), edge) < g_numericalToleranceSq:
                        intersectSpace.AddConfigLine(configLine2)

                    continue

                p11 = configLine1.GetPos1()
                p12 = configLine1.GetPos2()
                p21 = configLine2.GetPos1()
                p22 = configLine2.GetPos2()
                pe1 = v3f(p12[0] - p11[0], p12[1] - p11[1], 0.f)
                pe2 = v3f(p22[0] - p21[0], p22[1] - p21[1], 0.f)
                cp = cross(pe1, pe2)
                if mag2(cp) > g_numericalTolerance:
                    # Not parallel...
                    v2f pi
                    flagIntersect = SegmentIntersection(p11, p12, p21, p22, pi)
                    if flagIntersect == True:
                        CConfigLine intersectLine(pi)
                        intersectSpace.AddConfigLine(intersectLine)


                else:
                    # Parallel...
                    posMin1 = min_union(p11, p12)
                    posMax1 = max_union(p11, p12)
                    posMin2 = min_union(p21, p22)
                    posMax2 = max_union(p21, p22)
                    flagOverlap = True
                    for (j = 0; j < 2; j++)
                        if posMax1[j] < posMin2[j] - g_numericalTolerance or posMin1[j] > posMax2[j] + g_numericalTolerance:
                            flagOverlap = False
                            break


                    if flagOverlap == False:
                        continue

                    d1 = PointToLineSqDistance(p21, p12, p11)
                    d2 = PointToLineSqDistance(p22, p12, p11)
                    if d1 > g_numericalToleranceSq or d2 > g_numericalToleranceSq:
                        flagOverlap = False

                    if flagOverlap == False:
                        continue

                    v2f p1, p2
                    for (d = 0; d < 2; d++)
                        p1[d] = max(min(p11[d], p12[d]), min(p21[d], p22[d]))
                        p2[d] = min(max(p11[d], p12[d]), max(p21[d], p22[d]))

                    CConfigLine intersectLine(p1, p2)
                    intersectSpace.AddConfigLine(intersectLine)

        return intersectSpace

    def FindUnion(self, configSpace, configLine):
        configSpaceNew = configSpace
        if configSpace.GetNumOfLines() == 0:
            configSpaceNew.AddConfigLine(configLine)
            return configSpaceNew

        mergeFlag = False
        for (i = 0; i < configSpace.GetNumOfLines(); i++)
            line = configSpaceNew.GetConfigLine(i)
            CRoomEdge edge1(line.GetPos1(), line.GetPos2())
            CRoomEdge edge2(configLine.GetPos1(), configLine.GetPos2())
            sqlength1 = edge1.GetSqLength()
            sqlength2 = edge2.GetSqLength()
            if sqlength1 >= g_numericalTolerance and sqlength2 >= g_numericalTolerance:
                cp = cross(edge1.GetDirection3D(), edge2.GetDirection3D())
                if mag2(cp) > g_numericalTolerance:
                    continue


            elif sqlength1 < g_numericalTolerance and sqlength2 > g_numericalTolerance:
                continue

            elif sqlength1 < g_numericalTolerance and sqlength2 < g_numericalTolerance:
                if mag2(edge1.GetPos1() - edge2.GetPos1()) < g_numericalTolerance:
                    mergeFlag = True
                    break

                continue

            if PointToSegmentSqDistance(edge1.GetPos1(), edge2) < g_numericalToleranceSq or PointToSegmentSqDistance(edge1.GetPos2(), edge2) < g_numericalToleranceSq:
                posMin1 = min_union(edge1.GetPos1(), edge1.GetPos2())
                posMax1 = max_union(edge1.GetPos1(), edge1.GetPos2())
                posMin2 = min_union(edge2.GetPos1(), edge2.GetPos2())
                posMax2 = max_union(edge2.GetPos1(), edge2.GetPos2())
                posMin = min_union(posMin1, posMin2)
                posMax = max_union(posMax1, posMax2)
                v2f pos1, pos2
                for (j = 0; j < 2; j++)
                    pos1[j] = (line.GetPos1()[j] == posMin1[j]) ? posMin[j] : posMax[j]
                    pos2[j] = (line.GetPos1()[j] == posMin1[j]) ? posMax[j] : posMin[j]

                line.SetPos1(pos1)
                line.SetPos2(pos2)
                mergeFlag = True
                break

        if mergeFlag == False:
            configSpaceNew.AddConfigLine(configLine)

        return configSpaceNew

    def SelfMerge(self):
        sort(m_vecConfigLine.begin(), m_vecConfigLine.end(), CompareConfigLineLength)
        CConfigSpace configSpaceNew
        for (i = 0; i < GetNumOfLines(); i++)
            configSpaceNew = FindUnion(configSpaceNew, GetConfigLine(i))

        SetConfigLines(configSpaceNew.GetConfigLines())

    def GetConfigSpaceSize(self):
        sz = 0.f
        for (i = 0; i < GetNumOfLines(); i++)
            sz += GetConfigLine(i).get_config_line_length()

        return sz

    def PrintConfigSpace(self):
        for (i = 0; i < GetNumOfLines(); i++)
            std.cout << "The " << i << "th line:\n"
            GetConfigLine(i).PrintConfigLine()

    def translate_config_space(self, trans):
        for (i = 0; i < GetNumOfLines(); i++)
            GetConfigLine(i).translate_config_line(trans)

    def CompareConfigLineLength(self, line1, line2):
        return (line1.get_config_line_sq_length() > line2.get_config_line_sq_length())

    def PrecomputeTable(self, vecRooms):
        self.flag_precomputed = False
        self.precomputed_table.clear()
        numOfRooms = int(vecRooms.size())
        self.precomputed_table.resize(numOfRooms)
        #cout << "Pre-compute configuration space table for " << numOfRooms << " rooms...\n"
        for (i = 0; i < numOfRooms; i++)
            std.vector<CConfigSpace> vecConfigSpace(numOfRooms)
            room1 = vecRooms[i]
            centerPos = room1.get_room_center()
            room1.TranslateRoom(-centerPos)
            for (j = 0; j < numOfRooms; j++)
                vecConfigSpace[j] = CConfigSpace(room1, vecRooms[j])

            self.precomputed_table[i] = vecConfigSpace

        self.flag_precomputed = True
