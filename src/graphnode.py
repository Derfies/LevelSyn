import random

from reactor.geometry.vector import Vector2


class GraphNode:
    
    def __init__(self, g, node):
        self.g = g
        self.node = node
        if self.pos is None:
            self.randomly_init_pos()
        if self.flag_visited is None:
            self.flag_visited = False
        if self.flag_fixed is None:
            self.flag_fixed = False
        self.type = 0
        self.boundary_type = 0

    @property
    def flag_visited(self):
        return self.g.nodes[self.node].get('flag_visited')

    @flag_visited.setter
    def flag_visited(self, value):
        self.g.nodes[self.node]['flag_visited'] = value

    @property
    def flag_fixed(self):
        return self.g.nodes[self.node].get('flag_fixed')

    @flag_fixed.setter
    def flag_fixed(self, value):
        self.g.nodes[self.node]['flag_fixed'] = value

    @property
    def pos(self):
        return self.g.nodes[self.node].get('position')

    @pos.setter
    def pos(self, pos):
        self.g.nodes[self.node]['position'] = pos

    def randomly_init_pos(self):
        self.pos = Vector2(0, 0)
        for i in range(2):
            p = random.random()
            p -= 0.5
            p *= 1.5
            self.pos[i] = p

# 
# 

#     void SetPos(float px, py)
#         self.pos[0] = px
#         self.pos[1] = py
# 
# 
#     void ClearNeighbors() { self.neighbors.clear();
#     void AddNeighbor(int idx) { self.neighbors.push_back(idx);
#     std.vector<int>& GetNeighbors() { return self.neighbors;
#     bool IsNeighbor(int idx)
#         for (i = 0; i < int(self.neighbors.size()); i++)
#             if self.neighbors[i] == idx:
#                 return True
# 
# 
#         return False
# 
# 
#     bool Getflag_visited() { return self.flag_visited;
#     void Setflag_visited(bool flag_visited) { self.flag_visited = flag_visited;
# 
#     bool Getflag_fixed()  { return self.flag_fixed;
#     void Setflag_fixed(bool flag_fixed) { self.flag_fixed = flag_fixed;
# 
#     int GetType() { return self.type;
#     void SetType(int type) { self.type = type;
# 
#     int Getboundary_type()  { return self.boundary_type;
#     void Setboundary_type(int type) { self.boundary_type = type;
# 
# private:
#     std.string self.name
#     v2f self.pos
#     std.vector<int> self.neighbors
#     bool self.flag_visited
#     bool self.flag_fixed
#     int self.type; # index of the room template
#     int self.boundary_type
# 
# 
# #endif # GRAPHNODE_H
