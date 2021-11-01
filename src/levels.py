import random
import sys
import time

from configspace import ConfigSpace
from levelconfig import LevelConfig
from levelsync import LevelSynth
from planargraph import PlanarGraph
from roomtemplates import RoomTemplates


def main(argv):
    if len(argv) < 4:
        print(f'Usage: {argv[0]} graph.xml templates.xml config.txt [target_solution_number]')
        return -1

    level_config = LevelConfig()
    level_config.load_from_syn_config(argv[3])
    if len(argv) > 4:
        level_config.target_num_solutions = int(argv[4])

    planar_graph = PlanarGraph()
    planar_graph.load_graph_from_xml(argv[1])

    room_templates = RoomTemplates()
    room_templates.load_templates_from_xml(argv[2])

    if not level_config.flag_randomness:
        random.seed(0)

    if level_config.flag_enrich_templates:
        room_templates.enrich_by_rotating_180_degrees()

    config_space = ConfigSpace()
    config_space.precompute_table(room_templates.rooms)

    old_time = time.time()
    level_synthesizer = LevelSynth()
    level_synthesizer.set_graph_and_templates(planar_graph, room_templates)
    elapse_time = time.time() - old_time

    print(f'Have found {level_synthesizer.get_solutions_count()} solution(s) within {elapse_time} seconds.'

    return 0


if __name__ == '__main__':
    main(sys.argv)
