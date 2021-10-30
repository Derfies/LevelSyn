SCALING_FACTOR = 1e10


# def find_intersection(room1, room2):
#     Path p1, p2
#     p1.resize(room1.GetNumOfVertices())
#     p2.resize(room2.GetNumOfVertices())
#
#     for (i = 0; i < room1.GetNumOfVertices(); i++)
#         pi = room1.GetVertex(i)
#         p1[i].X = convert_float_to_long64(pi[0])
#         p1[i].Y = convert_float_to_long64(pi[1])
#
#     for (i = 0; i < room2.GetNumOfVertices(); i++)
#         pi = room2.GetVertex(i)
#         p2[i].X = convert_float_to_long64(pi[0])
#         p2[i].Y = convert_float_to_long64(pi[1])
#
#
#     ct = ctIntersection
#     pft = pftNonZero
#     Paths sub, clp, sol
#
#     Clipper c
#     c.AddPath(p1, ptSubject, True)
#     c.AddPath(p2, ptClip, True)
#     c.Execute(ct, sol, pft, pft)
#
#     return sol


def compute_collide_area(room1, room2):

    # TODO: Replace with bools intersect.
    return 0
    #
    # #sol = find_intersection(room1, room2)
    # collide_area = 0
    # for (i = 0; i < int(sol.size()); i++)
    #     a = Area(sol[i])
    #     collide_area += abs(convert_double_area_to_float(a))
    #
    # if collide_area < 10e-4:
    #     collide_area = 0
    #
    # return collide_area


def compute_room_area(room):
    return compute_collide_area(room, room)


def convert_float_to_long64(f):
    i = long64(f * SCALING_FACTOR + 0.5)
    return i


def convert_long64_to_float(i):
    f = float(i) / SCALING_FACTOR
    return f


def convert_double_area_to_float(a):
    a = a / double(SCALING_FACTOR * SCALING_FACTOR)
    f = float(a)
    return f

