import random

from reactor.geometry.graph import Graph
from reactor.geometry.vector import Vector2


class PlanarGraph(Graph):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.num_types = 1

    @property
    def num_nodes(self):
        return len(self.nodes)

    def get_node(self, idx):
        return list(self)[idx]

    def get_node_pos(self, idx):
        return self.get_node(idx).get_pos()

    def get_fixed_nodes(self):
        indices = []
        for i in range(self.num_nodes):
            if self.get_node(i).get_flag_fixed():
                indices.append(i)
        return indices

    def get_graph_bounding_box(self):
        pos_min, pos_max = Vector2(1e10, 1e10), Vector2(-1e10, -1e10)
        for i in range(self.num_nodes):
            pi = self.get_node(i).get_pos()
            for j in range(2):
                pos_min[j] = min(pos_min[j], pi[j])
                pos_max[j] = max(pos_max[j], pi[j])
        return pos_min, pos_max

    def move_graph_to_scene_centre(self):
        pos_min, pos_max = self.get_graph_bounding_box()
        pos_cen = (pos_min + pos_max) * 0.5
        for i in range(self.num_nodes):
            pi = self.get_node(i).get_pos()
            pi = pi - pos_cen
            self.get_node(i).set_pos(pi)

    def scale_graph_node_positions(self, scaling):
        if scaling <= 0:
            return
        for i in range(self.num_nodes):
            pi = self.get_node(i).get_pos()
            pi = pi * scaling
            self.get_node(i).set_pos(pi)

    def random_init_types(self):
        for i in range(self.num_nodes):
            type_idx = random.randint(0, self.num_types + 1)
            self.get_node(i).set_type(type_idx)

    def visited_no_node(self):
        for i in range(self.num_nodes):
            if self.get_node(i).get_flag_visited():
                return False
        return True

    def visited_all_nodes(self):
        for i in range(self.num_nodes):
            if not self.get_node(i).get_flag_visted():
                return False
        return True

    def count_constraints(self, indices):
        count = 0
        for i in range(len(indices)):
            idx = indices[i]
            neighbors = self.get_node(idx).get_neighbours()
            for j in range(len(neighbors)):
                idx_tmp = neighbors[j]
                if self.get_node(idx_tmp).get_flag_visited:
                    count += 1
                    break
        return count

    def has_fixed_node(self):
        for i in range(self.num_nodes):
            if self.get_node(i).get_flag_fixed():
                return True
        return False

    def extract_deepest_face_or_chain(self, flag_cyclic, flag_small_face_first):
        if self.visited_no_node() and self.has_fixed_node:
            # Handle the fixed nodes first...
            indices = self.get_fixed_nodes()
            return indices

        if self.chains:
            for i in range(len(self.chains)):
                chain = self.get_chain(i)
# if 0 # Before 09/23/2013, is NOT suitable for ghost cases
                idx0 = chain.get_indices()[0]
                if self.get_node(idx0).get_flag_visited():
                    continue

# else:
                flag_all_visited = True
                for j in range(chain.size):
                    if not self.get_node(chain.get_indices()[j]).get_flag_visited():
                        flag_all_visited = False
                        break

                if flag_all_visited:
                    continue

# endif
                flag_cyclic = chain.get_flag_cyclic()
                return chain.get_indices()


        face_indices = self.extract_deepest_face(flag_small_face_first)
        if face_indices.empty() == False and self.visited_no_node():
            flag_cyclic = True
            return face_indices
        
        face_constraint_count = self.count_constraints(face_indices)
        chain_indices = self.extract_deepest_chain_new()
        chain_constraint_count = self.count_constraints(chain_indices)
        if face_indices.empty() == False and face_constraint_count > chain_constraint_count:
            flag_cyclic = True
            return face_indices
        
        else:
            flag_cyclic = False
            return chain_indices
