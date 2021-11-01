import pyclipper


SCALING_FACTOR = 1e10


def find_intersection(room1, room2):
    p1 = []
    p2 = []
    for i in range(len(room1.num_vertices)):
        pi = room1.vertex[i]
        p1[i].X = convert_float_to_long64(pi[0])
        p1[i].Y = convert_float_to_long64(pi[1])

    for i in range(len(room2.num_vertices)):
        pi = room2.vertex[i]
        p2[i].X = convert_float_to_long64(pi[0])
        p2[i].Y = convert_float_to_long64(pi[1])


    ct = pyclipper.CT_INTERSECTION
    pft = pyclipper.PFT_NON_ZERO
    Paths sub, clp, sol

    pc = pyclipper.Pyclipper()
    pc.AddPath(p1, pyclipper.PT_SUBJECT, True)
    pc.AddPath(p2, pyclipper.PT_CLIP, True)
    pc.Execute(ct, sol, pft, pft)

    return sol


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

