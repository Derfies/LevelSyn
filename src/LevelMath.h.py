#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-06-15
#  email:  chongyangm@gmail.com
#  info: wrapper for basic math operations
# --------------------------------------------------------------

#ifndef LEVELMATH_H
#define LEVELMATH_H

#include "Room.h"

namespace level_math
typedef struct PrSort
    v2f m_pr
    float m_dp; # dot product
} PrSort

 g_numericalTolerance = 1e-4f; #1e-6

 g_numericalToleranceSq = g_numericalTolerance * g_numericalTolerance

def PointToSegmentSqDistance(self, pt, line):

def PointToLineSqDistance(self, pt, line):

def PointToLineSqDistance(self, pt, p1, p2):

def RoomPerimeter(self, room1):

def RoomContact(self, room1, room2):

def RoomContact(self, room1, room2, edgeIdx1, edgeIdx2):

def EdgeContact(self, line1, line2):

def RoomDistance(self, room1, room2):

def SegmentIntersection(self, pa, pb, pc, pd, pi):

def SegmentIntersection(self, Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):

def LineIntersection(self, pa, pb, pc, pd, pi):

def LineIntersection(self, Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, Ix, Iy):

def ComparePrSmallerFirst(self, pr1, pr2):

def SortVecPr(self, vecPr):

def randomColorFromIndex(self, idx):
} # namespace level_math

#endif # LEVELMATH_H
