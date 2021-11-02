import pyclipper

# subj = (
#     ((180, 200), (260, 200), (260, 150), (180, 150)),
#     ((215, 160), (230, 190), (200, 190))
# )
# subj = (((2.2, 2.2), (2, -2), (-2, -2), (-2, 2),),)
# clip = ((190, 210), (240, 210), (240, 130), (190, 130))

subj = [(2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0)]
clip = [(6.0, -2.0000004999999996), (6.0, -6.0000005), (2.0, -6.0000005), (2.0, -2.0000004999999996)]

pc = pyclipper.Pyclipper()
pc.AddPath(clip, pyclipper.PT_CLIP, True)
pc.AddPath(subj, pyclipper.PT_SUBJECT, True)

solution = pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
print(solution)