import numpy as np
import pyclipper


SCALING_FACTOR = 1e10


def find_intersection(room1, room2):
    p1 = [node.position for node in room1.get_nodes()]
    p2 = [node.position for node in room2.get_nodes()]

    ct = pyclipper.CT_INTERSECTION
    pft = pyclipper.PFT_NONZERO

    pc = pyclipper.Pyclipper()
    pc.AddPath(p1, pyclipper.PT_SUBJECT, True)
    pc.AddPath(p2, pyclipper.PT_CLIP, True)
    result = pc.Execute(ct, pft, pft)
    return result


def compute_collide_area(room1, room2):
    solutions = find_intersection(room1, room2)
    collide_area = 0
    for solution in solutions:
        a = pyclipper.Area(solution)
        collide_area += abs(convert_double_area_to_float(a))
    if collide_area < 10e-4:
        collide_area = 0
    return collide_area


def compute_room_area(room):
    return compute_collide_area(room, room)


def convert_float_to_long64(f):
    i = f * SCALING_FACTOR + 0.5
    return i


def convert_long64_to_float(i):
    f = float(i) / SCALING_FACTOR
    return f


def convert_double_area_to_float(a):
    a = a / SCALING_FACTOR * SCALING_FACTOR
    f = float(a)
    return f

