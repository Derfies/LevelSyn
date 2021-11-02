import json


class LevelConfig:

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    def __init__(self):
        self.FLAG_RANDOMNESS = False
        self.FLAG_ENABLE_TYPE_CHANGE = True
        self.FLAG_ENRICH_TEMPLATES = False
        self.FLAG_EQUAL_PICK_PROBABILITY = True
        self.FLAG_DISCRETE_CONNECTIVITY_FUNCTION = True
        self.FLAG_RANDOM_PICK = True
        self.FLAG_NON_OVERLAP_CONTACT = False
        self.FLAG_SMALL_FACE_FIRST = False
        self.FLAG_USE_ILS = False
        self.FLAG_RANDOM_WALK = False
        self.NUMBER_OF_SOLUTIONS_TO_TRACK = 10
        self.SYNTHESIS_METHOD = 0
        self.SA_NUM_OF_CYCLES = 1000
        self.SA_NUM_OF_TRIALS = 1000
        self.TARGET_NUM_SOLUTIONS = 100
        self.SA_PROB_0 = 0.001
        self.SA_PROB_1 = 0.7
        self.DELTA_E_SCALING = 1.0
        self.SIGMA_COLLIDE = 50.
        self.SIGMA_CONTACT = 1.0
        self.SIGMA_CONNECTIVITY = 2.0
        self.GRAPH_SCALING = 1.0
        self.ROOM_SCALING = 0.9
        self.STATE_DIFFERENCE_THRESHOLD = 0.0
        self.ROOM_CONTACT_THRESHOLD = 1e-6
        self.OUTPUT_PREFIX = ''

    def load_from_syn_config(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        for k, v in data.items():
            setattr(self, k, v)
