import json


class LevelConfig:

    _instance = None

    FLAG_RANDOMNESS = False
    FLAG_ENABLE_TYPE_CHANGE = True
    FLAG_ENRICH_TEMPLATES = False
    FLAG_EQUAL_PICK_PROBABILITY = True
    FLAG_DISCRETE_CONNECTIVITY_FUNCTION = True
    FLAG_RANDOM_PICK = True
    FLAG_NON_OVERLAP_CONTACT = False
    FLAG_SMALL_FACE_FIRST = False
    FLAG_USE_ILS = False
    FLAG_RANDOM_WALK = False
    NUMBER_OF_SOLUTIONS_TO_TRACK = 10
    SYNTHESIS_METHOD = 0
    SA_NUM_OF_CYCLES = 2
    SA_NUM_OF_TRIALS = 2
    TARGET_NUM_SOLUTIONS = 100
    SA_PROB_0 = 0.001
    SA_PROB_1 = 0.7
    DELTA_E_SCALING = 1.0
    SIGMA_COLLIDE = 5.0
    SIGMA_CONTACT = 1.0
    SIGMA_CONNECTIVITY = 2.0
    GRAPH_SCALING = 1.0
    ROOM_SCALING = 0.9
    STATE_DIFFERENCE_THRESHOLD = 0.0
    ROOM_CONTACT_THRESHOLD = 1e-6
    OUTPUT_PREFIX = ''

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def load_from_syn_config(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        for k, v in data.items():
            setattr(self, k, v)
            #print(k, '->', v)
        #print('here:', self.FLAG_USE_ILS)