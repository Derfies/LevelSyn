#include "LevelConfig.h"

#ifndef WIN32
    #define MAX_PATH 260
#endif

bool CLevelConfig.m_flagRandomness = False
bool CLevelConfig.m_flagEnableTypeChange = True
bool CLevelConfig.m_flagEnrichTemplates = False
bool CLevelConfig.m_flagEqualPickProb = True
bool CLevelConfig.m_flagDiscreteConnectFunc = True
bool CLevelConfig.m_flagRandomPick = True
bool CLevelConfig.m_flagNonOverlapContact = False
bool CLevelConfig.m_flagSmallFaceFirst = False
bool CLevelConfig.m_flagUseILS = False
bool CLevelConfig.m_flagRandomWalk = False
int CLevelConfig.m_numOfSolutionsToTrack = 10
int CLevelConfig.m_synMethod = 0
int CLevelConfig.m_saNumOfCycles = 1000
int CLevelConfig.m_saNumOfTrials = 1000
int CLevelConfig.m_targetNumOfSolutions = 100
float CLevelConfig.m_saProb0 = 0.001f
float CLevelConfig.m_saProb1 = 0.7f
float CLevelConfig.m_deltaEscaling = 1.0f
float CLevelConfig.m_sigmaCollide = 50.f
float CLevelConfig.m_sigmaContact = 1.f
float CLevelConfig.m_sigmaConnectivity = 2.f
float CLevelConfig.m_graphScaling = 1.f
float CLevelConfig.m_roomScaling = 0.9f
float CLevelConfig.m_stateDiffThresh = 0.f
float CLevelConfig.m_roomContactThresh = 1e-6f
std.string CLevelConfig.m_outputPrefix

CLevelConfig.CLevelConfig()


def LoadFromSynConfig(self, fileName, resetFlag ''' = True '''):
    std.ifstream fin(fileName.c_str())
    if fin.fail() == True:
        std.cout << "Failed to load config parameters from config file " << fileName << "not \n"
        return False

    std.string param
    while (fin >> param)
        if param == std.string("FLAG_RANDOMNESS"):
            fin >> m_flagRandomness

        elif param == std.string("FLAG_ENABLE_TYPE_CHANGE"):
            fin >> m_flagEnableTypeChange

        elif param == std.string("FLAG_ENRICH_TEMPLATES"):
            fin >> m_flagEnrichTemplates

        elif param == std.string("FLAG_EQUAL_PICK_PROBABILITY"):
            fin >> m_flagEqualPickProb

        elif param == std.string("FLAG_DISCRETE_CONNECTIVITY_FUNCTION"):
            fin >> m_flagDiscreteConnectFunc

        elif param == std.string("FLAG_RANDOM_PICK"):
            fin >> m_flagRandomPick

        elif param == std.string("FLAG_NON_OVERLAP_CONTACT"):
            fin >> m_flagNonOverlapContact

        elif param == std.string("FLAG_SMALL_FACE_FIRST"):
            fin >> m_flagSmallFaceFirst

        elif param == std.string("FLAG_USE_ILS"):
            fin >> m_flagUseILS

        elif param == std.string("FLAG_RANDOM_WALK"):
            fin >> m_flagRandomWalk

        elif param == std.string("NUMBER_OF_SOLUTIONS_TO_TRACK"):
            fin >> m_numOfSolutionsToTrack

        elif param == std.string("SYNTHESIS_METHOD"):
            fin >> m_synMethod

        elif param == std.string("SA_NUM_OF_CYCLES"):
            fin >> m_saNumOfCycles

        elif param == std.string("SA_NUM_OF_TRIALS"):
            fin >> m_saNumOfTrials

        elif param == std.string("SA_PROB_0"):
            fin >> m_saProb0

        elif param == std.string("SA_PROB_1"):
            fin >> m_saProb1

        elif param == std.string("DELTA_E_SCALING"):
            fin >> m_deltaEscaling

        elif param == std.string("SIGMA_COLLIDE"):
            fin >> m_sigmaCollide

        elif param == std.string("SIGMA_CONTACT"):
            fin >> m_sigmaContact

        elif param == std.string("SIGMA_CONNECTIVITY"):
            fin >> m_sigmaConnectivity

        elif param == std.string("GRAPH_SCALING"):
            fin >> m_graphScaling

        elif param == std.string("ROOM_SCALING"):
            fin >> m_roomScaling

        elif param == std.string("STATE_DIFFERENCE_THRESHOLD"):
            fin >> m_stateDiffThresh

        elif param == std.string("ROOM_CONTACT_THRESHOLD"):
            fin >> m_roomContactThresh

        elif param == std.string("OUTPUT_PREFIX"):
            fin >> m_outputPrefix


    if resetFlag == False:
        return True

    ResetConfig()

    return True


def AddOutputPrefix(self, str):
    if m_outputPrefix.empty() == True:
        return str

    std.ostringstream oss
    oss << m_outputPrefix << str
    strNew = oss.str()
    return strNew


def ResetConfig(self):
    if m_flagRandomness == True:
        srand((unsigned int)time(0))

    if m_outputPrefix.empty() == False:
        UpdateOutputPrefix()

    DumpToSynConfig()


def DumpToSynConfig(self):
    std.ostringstream oss
    oss << m_outputPrefix << "SynConfig.txt"
    fileName = oss.str()
    FILE* file
    file = fopen(fileName.c_str(), "w")
    if not file:
        std.cout << "Failed to dump parameters into config file " << fileName << "not \n"
        return False

    DumpTimeAndDate(file)
    DumpParameters(file)
    fclose(file)

    return True


def DumpTimeAndDate(self, file):
    myTime = time(NULL)
    ptrTime = localtime(&myTime)
    fprintf(file, "%02d:%02d:%02d ", ptrTime.tm_hour, ptrTime.tm_min, ptrTime.tm_sec)
    fprintf(file, "%02d/%02d/%04d\n\n", ptrTime.tm_mon + 1, ptrTime.tm_mday, ptrTime.tm_year + 1900)


def DumpTimeAndDate(self, fout):
    myTime = time(NULL)
    ptrTime = localtime(&myTime)
    char str[1000]
    sprintf(str, "%02d:%02d:%02d ", ptrTime.tm_hour, ptrTime.tm_min, ptrTime.tm_sec)
    fout << str
    sprintf(str, "%02d/%02d/%04d", ptrTime.tm_mon + 1, ptrTime.tm_mday, ptrTime.tm_year + 1900)
    fout << str << std.endl


def DumpParameters(self, file):
    fprintf(file, "%s\t%d\n", "FLAG_RANDOMNESS", m_flagRandomness)
    fprintf(file, "%s\t%d\n", "FLAG_ENABLE_TYPE_CHANGE", m_flagEnableTypeChange)
    fprintf(file, "%s\t%d\n", "FLAG_ENRICH_TEMPLATES", m_flagEnrichTemplates)
    fprintf(file, "%s\t%d\n", "FLAG_EQUAL_PICK_PROBABILITY", m_flagEqualPickProb)
    fprintf(file, "%s\t%d\n", "FLAG_DISCRETE_CONNECTIVITY_FUNCTION", m_flagDiscreteConnectFunc)
    fprintf(file, "%s\t%d\n", "FLAG_RANDOM_PICK", m_flagRandomPick)
    fprintf(file, "%s\t%d\n", "FLAG_NON_OVERLAP_CONTACT", m_flagNonOverlapContact)
    fprintf(file, "%s\t%d\n", "FLAG_SMALL_FACE_FIRST", m_flagSmallFaceFirst)
    fprintf(file, "%s\t%d\n", "FLAG_USE_ILS", m_flagUseILS)
    fprintf(file, "%s\t%d\n", "FLAG_RANDOM_WALK", m_flagRandomWalk)
    fprintf(file, "%s\t%d\n", "NUMBER_OF_SOLUTIONS_TO_TRACK", m_numOfSolutionsToTrack)
    fprintf(file, "%s\t%d\n", "SYNTHESIS_METHOD", m_synMethod)
    fprintf(file, "%s\t%d\n", "SA_NUM_OF_CYCLES", m_saNumOfCycles)
    fprintf(file, "%s\t%d\n", "SA_NUM_OF_TRIALS", m_saNumOfTrials)
    fprintf(file, "%s\t%f\n", "SA_PROB_0", m_saProb0)
    fprintf(file, "%s\t%f\n", "SA_PROB_1", m_saProb1)
    fprintf(file, "%s\t%f\n", "DELTA_E_SCALING", m_deltaEscaling)
    fprintf(file, "%s\t%f\n", "SIGMA_COLLIDE", m_sigmaCollide)
    fprintf(file, "%s\t%f\n", "SIGMA_CONTACT", m_sigmaContact)
    fprintf(file, "%s\t%f\n", "SIGMA_CONNECTIVITY", m_sigmaConnectivity)
    fprintf(file, "%s\t%f\n", "GRAPH_SCALING", m_graphScaling)
    fprintf(file, "%s\t%f\n", "ROOM_SCALING", m_roomScaling)
    fprintf(file, "%s\t%f\n", "STATE_DIFFERENCE_THRESHOLD", m_stateDiffThresh)
    fprintf(file, "%s\t%f\n", "ROOM_CONTACT_THRESHOLD", m_roomContactThresh)
    DumpStringParam(file, "OUTPUT_PREFIX", m_outputPrefix)


def UpdateOutputPrefix(self):
#ifdef WIN32
    if m_outputPrefix[m_outputPrefix.size() - 1] != '\\':
        m_outputPrefix = m_outputPrefix + std.string("\\")

#else:
    while (m_outputPrefix.empty() == False and isdigit(m_outputPrefix[m_outputPrefix.size() - 1]) == 0)
        m_outputPrefix = m_outputPrefix.substr(0, m_outputPrefix.size() - 1)

    if m_outputPrefix[m_outputPrefix.size() - 1] != '/':
        m_outputPrefix = m_outputPrefix + std.string("/")

#endif
    std.ostringstream oss
#ifdef WIN32
    flag = CreateDirectoryA(m_outputPrefix.c_str(), NULL)
     numLength = 2
    while (flag == False and m_outputPrefix.size() >= 2)
        subStr = m_outputPrefix.substr(m_outputPrefix.length() - numLength - 1, 2)
        num = atoi(subStr.c_str()) + 1
        char numChar[MAX_PATH]
        sprintf_s(numChar, "%02d", num)
        std.ostringstream oss
        oss << m_outputPrefix.substr(0, m_outputPrefix.length() - numLength - 1) << numChar << "\\"
        m_outputPrefix = oss.str()
        flag = CreateDirectoryA(m_outputPrefix.c_str(), NULL)

#else:
    outputFolder = m_outputPrefix.substr(0, m_outputPrefix.size() - 1)
    flag = mkdir(outputFolder.c_str(), 0777)
     numLength = 2
    while (flag == -1 and m_outputPrefix.size() >= 2)
        subStr = m_outputPrefix.substr(m_outputPrefix.length() - numLength - 1, 2)
        num = atoi(subStr.c_str()) + 1
        char numChar[MAX_PATH]
        sprintf(numChar, "%02d", num)
        std.ostringstream oss
        oss << m_outputPrefix.substr(0, m_outputPrefix.length() - numLength - 1) << numChar << "/"
        m_outputPrefix = oss.str()
        outputFolder = m_outputPrefix.substr(0, m_outputPrefix.size() - 1)
        flag = mkdir(outputFolder.c_str(), 0777)

#endif
    std.cout << "Generating results into the directory: " << m_outputPrefix.c_str() << "...\n"


def DumpStringParam(self, file, param, str):
    if str.empty() != True:
        fprintf(file, "%s\t%s\n", param, str.c_str())

    else:
        #fprintf(file, "%s\t%s\n", param, "NULL")


