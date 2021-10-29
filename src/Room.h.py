#  Copyright (c) www.chongyangma.com
#
#  author: Chongyang Ma - 2013-03-07
#  email:  chongyangm@gmail.com
#  info: class declaration of a single room
# --------------------------------------------------------------

#ifndef ROOM_H
#define ROOM_H

#include <iostream>
#include <ctime>
#include <cmath>
#include <cstring>
#include <vector>

#include "tinyxml2.h"

#include "vec.h"
#include "RoomEdge.h"

typedef struct AABB2f
	v2f m_posMin
	v2f m_posMax
} AABB2f

typedef CLineBase RoomWall
typedef CLineBase RoomDoor

class CRoom
public:
	CRoom()

	std.vector<v2f>& GetVertices() { return m_vertices;

	void SetVertices(std.vector<v2f>& vertices) { m_vertices = vertices;

	void SetVertex(v2f& pos, idx) { m_vertices[idx] = pos;

	void SetCenterShift(v2f& shift) { m_centerShift = shift;

	v2f GetCenterShift()  { return m_centerShift;

	v2f GetVertex(int idx)  { return m_vertices[idx];

	CRoomEdge GetEdge(int idx)

	int GetNumOfVertices()  { return int(m_vertices.size());

	int GetNumOfEdges()  { return int(m_vertices.size());

	v2f GetRoomCenter()

	v2f GetShiftedRoomCenter()

	void TranslateRoom(v2f trans)

	void RotateRoom(float rad)

	void ScaleRoom(float scaling)

	void ScaleRoom(v2f scaling)

	void GetRoomBoundingBox(v2f& posMin, posMax)

	void GetRoomBoundingBox(AABB2f& boundingBox)

	v3f GetColor() { return m_color;
	void SetColor(v3f color) { m_color = color;

	float GetEnergy() { return m_energy;
	void SetEnergy(float energy) { m_energy = energy;
	void ResetEnergy() { SetEnergy(1.f);
	void UpdateEnergy(float factor) { m_energy *= factor;

	void PrintRoom()

	bool HasWalls()  { return (m_walls.empty() == False);

	void InitWalls()

	bool EraseWall(int idx)

	void InsertWall(RoomWall& wall) { m_walls.push_back(wall);

	int GetNumOfWalls()  { return int(m_walls.size());

	RoomWall& GetWall(int idx) { return m_walls[idx];

	int GetTemplateType()  { return m_templateType;
	void SetTemplateType(int type) { m_templateType = type;

	int GetBoundaryType()  { return m_boundaryType;
	void SetBoundaryType(int type) { m_boundaryType = type;

	bool GetFlagFixed()  { return m_flagFixed;
	void SetFlagFixed(bool flagFixed) { m_flagFixed = flagFixed;

	void ResetDoorFlags()
	void SetDoorFlag(int edgeIdx, doorFlag)
	bool GetDoorFlag(int edgeIdx)
	std.vector<bool> GetDoorFlags()
	bool HasRestrictedDoorPosition()

private:
	std.vector<v2f> m_vertices
	m_centerShift = v2f(0.f, 0.f);
	std.vector<RoomWall> m_walls
	m_color = v3f(0.5f, 0.5f, 0.5f)
	float m_energy
	int m_templateType
	bool m_flagFixed
	int m_boundaryType
	std.vector<bool> m_doorFlags


#endif # ROOM_H
