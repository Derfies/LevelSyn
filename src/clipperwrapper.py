import numpy as np
import pyclipper


SCALING_FACTOR = 1e10


def find_intersection(room1, room2):
    p1 = []
    p2 = []
    for i in range(room1.num_vertices):
        pi = room1.vertices[i]
        p1.append((int(pi.x), int(pi.y)))
        #p1[i].x = convert_float_to_long64(pi[0])
        #p1[i].y = convert_float_to_long64(pi[1])

    for i in range(room2.num_vertices):
        pi = room2.vertices[i]
        p2.append((int(pi.x), int(pi.y)))
        #p2[i].x = convert_float_to_long64(pi[0])
        #p2[i].y = convert_float_to_long64(pi[1])

    ct = pyclipper.CT_INTERSECTION
    pft = pyclipper.PFT_NONZERO

    print(p1)
    print(p2)


    pc = pyclipper.Pyclipper()
    pc.AddPath(p1, pyclipper.PT_SUBJECT, False)
    pc.AddPath(p2, pyclipper.PT_CLIP, False)

    try:
        result = pc.Execute(ct, pft, pft)
    except Exception as e:
        print(e)
        result = []
    else:
        print('clipping worked!')


    return result


def compute_collide_area(room1, room2):
    sol = find_intersection(room1, room2)
    collide_area = 0
    #for (int i = 0; i < int(sol.size()); i++)
    for i in range(len(sol)):
        a = Area(sol[i])
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

