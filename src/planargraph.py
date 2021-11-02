import sys
import random

from reactor.geometry.graph import Graph
from reactor.geometry.vector import Vector2
from reactor.faceanalysis import FaceAnalysis
from graphface import GraphFace
from graphnode import GraphNode


class PlanarGraph(Graph):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.num_types = 1
        self.chains = []
        self.faces = []

    @classmethod
    def load(cls, *args, **kwargs):
        g = super().load(*args, **kwargs)
        nodes = list(g)
        g.faces = []
        for face in FaceAnalysis(g).get_faces():
            indices = [idx for idx, node in enumerate(nodes) if node in face]
            g.faces.append(GraphFace(g, indices))
        return g

    @property
    def num_nodes(self):
        return len(self.nodes)

    @property
    def num_faces(self):
        return len(self.faces)

    def get_node(self, idx):
        return GraphNode(self, list(self)[idx])

    def get_node_pos(self, idx):
        return self.get_node(idx).pos

    def get_fixed_nodes(self):
        indices = []
        for i in range(self.num_nodes):
            if self.get_node(i).flag_fixed:
                indices.append(i)
        return indices

    def get_face(self, idx):
        return self.faces[idx]#GraphFace(self, self.faces[idx])

    def get_graph_bounding_box(self):
        pos_min, pos_max = Vector2(1e10, 1e10), Vector2(-1e10, -1e10)
        for i in range(self.num_nodes):
            pi = self.get_node(i).pos
            for j in range(2):
                pos_min[j] = min(pos_min[j], pi[j])
                pos_max[j] = max(pos_max[j], pi[j])
        return pos_min, pos_max

    def move_graph_to_scene_centre(self):
        pos_min, pos_max = self.get_graph_bounding_box()
        pos_cen = (pos_min + pos_max) * 0.5
        for i in range(self.num_nodes):
            pi = self.get_node(i).pos
            pi = pi - pos_cen
            self.get_node(i).pos = pi

    def scale_graph_node_positions(self, scaling):
        if scaling <= 0:
            return
        for i in range(self.num_nodes):
            pi = self.get_node(i).pos
            pi = pi * scaling
            self.get_node(i).pos = pi

    def random_init_types(self):
        for i in range(self.num_nodes):
            type_idx = random.randint(0, self.num_types + 1)
            self.get_node(i).type = type_idx

    def visited_no_node(self):
        for i in range(self.num_nodes):
            if self.get_node(i).flag_visited:
                return False
        return True

    def visited_all_nodes(self):
        for i in range(self.num_nodes):
            if not self.get_node(i).flag_visited:
                return False
        return True

    def count_constraints(self, indices):
        count = 0
        for i in range(len(indices)):
            idx = indices[i]
            neighbors = self.get_node(idx).get_neighbours()
            for j in range(len(neighbors)):
                idx_tmp = neighbors[j]
                if self.get_node(idx_tmp).flag_visited:
                    count += 1
                    break
        return count

    def has_fixed_node(self):
        for i in range(self.num_nodes):
            if self.get_node(i).flag_fixed:
                return True
        return False

    def extract_deepest_face_or_chain(self, flag_cyclic, flag_small_face_first):
        if self.visited_no_node() and self.has_fixed_node():
            # Handle the fixed nodes first...
            indices = self.get_fixed_nodes()
            return indices

        if self.chains:
            for i in range(len(self.chains)):
                chain = self.get_chain(i)
# if 0 # Before 09/23/2013, is NOT suitable for ghost cases
#                 idx0 = chain.get_indices()[0]
#                 if self.get_node(idx0).flag_visited:
#                     continue

# else:
                flag_all_visited = True
                for j in range(chain.size):
                    if not self.get_node(chain.get_indices()[j]).flag_visited:
                        flag_all_visited = False
                        break

                if flag_all_visited:
                    continue

# endif
                flag_cyclic = chain.get_flag_cyclic()
                return chain.get_indices()


        face_indices = self.extract_deepest_face(flag_small_face_first)
        if face_indices and self.visited_no_node():
            flag_cyclic = True
            return face_indices
        
        face_constraint_count = self.count_constraints(face_indices)
        chain_indices = self.extract_deepest_chain_new()
        chain_constraint_count = self.count_constraints(chain_indices)
        if face_indices and face_constraint_count > chain_constraint_count:
            flag_cyclic = True
            return face_indices
        
        else:
            flag_cyclic = False
            return chain_indices

    def extract_deepest_face(self, flag_small_face_first):
        face_indices = []
        deepest_face_idx = -1
        deepest_face_size = 0
        smallest_face_idx = -1
        smallest_face_size = sys.maxsize
        for i in range(self.num_faces):
            face = self.get_face(i)
            indices = face.indices
            flag_all_nodes_unvisited = True
            for j in range(face.size):
                if self.get_node(indices[j]).flag_visited:
                    flag_all_nodes_unvisited = False
                    break

            if not flag_all_nodes_unvisited:
                continue

            if face.size > deepest_face_size:
                deepest_face_idx = i
                deepest_face_size = face.size

            if face.size < smallest_face_size:
                smallest_face_idx = i
                smallest_face_size = face.size

        if deepest_face_idx >= 0:
            face_indices = self.get_face(deepest_face_idx).indices

        if flag_small_face_first and smallest_face_idx >= 0:
            face_indices = self.get_face(smallest_face_idx).indices

        return face_indices

    def extract_deepest_chain_new(self):
        chain_length_min = sys.maxsize
        chain_length_max = 0
        chain_length_min_c = 0
        chain_length_max_c = 0
        chain_min = []
        chain_max = []
        for i in range(self.num_nodes):
            if self.get_node(i).flag_visted:
                continue

            chain_indices = []
            idx = i
            chain_indices.append(idx)
            self.get_node(idx).flag_visited = True
            # Grow the chain with unvisited nodes...
            flag_inserted = True
            while flag_inserted:
                flag_inserted = False
                neighbours = self.get_node(idx).get_neigbours()
                for j in range(len(neighbours)):
                    idx_tmp = neighbours[j]
                    if not self.get_node(idx_tmp).flag_visited:
                        idx = idx_tmp
                        chain_indices.append(idx)
                        self.get_node(idx).flag_visited = True
                        flag_inserted = True
                        break

            chain_length_tmp = int(chain_indices.size())
            # Set the visited flags back to False...
            for j in range(len(chain_length_tmp)):
                self.get_node(chain_indices[j]).flag_visited = False

            chain_constraint_count = self.count_constraints(chain_indices)
            if chain_constraint_count >= chain_length_min_c and chain_length_tmp < chain_length_min:
                chain_length_min_c = chain_constraint_count
                chain_length_min = chain_length_tmp
                chain_min = chain_indices

            if chain_constraint_count >= chain_length_max_c and chain_length_tmp > chain_length_max:
                chain_length_max_c = chain_constraint_count
                chain_length_max = chain_length_tmp
                chain_max = chain_indices

        return chain_min
