ROOM_CONTACT_THRESH = 1e-6


class LevelConfig:
    
    def __init__(self):
        self.flag_randomness = False
        self.flag_enable_type_change = True
        self.flag_enrich_templates = False
        self.flag_equal_pick_prob = True
        self.flag_discrete_connect_func = True
        self.flag_random_pick = True
        self.flag_non_overlap_contact = False
        self.flag_small_face_first = False
        self.flag_use_ils = False
        self.flag_random_walk = False
        self.num_solutions_to_track = 10
        self.syn_method = 0
        self.sa_num_of_cycles = 1000
        self.sa_num_of_trials = 1000
        self.target_num_solutions = 100
        self.sa_prob0 = 0.001
        self.sa_prob1 = 0.7
        self.delta_escaling = 1.0
        self.sigma_collide = 50.
        self.sigma_contact = 1.0
        self.sigma_connectivity = 2.0
        self.graph_scaling = 1.0
        self.room_scaling = 0.9
        self.state_diff_thresh = 0.0
        self.room_contact_thresh = 1e-6
