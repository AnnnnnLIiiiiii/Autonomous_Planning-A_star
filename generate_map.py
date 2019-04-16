import cv2
import numpy as np


def generate_map():

    while 1:
        r = int(input("Enter the radius of cycle robot: "))
        if r >= 0: break
        else: print("Incorrect radius of cycle robot!")
    while 1:
        clearance = int(input("Enter the clearance between robot and obstacles: "))
        if clearance >= 0: break
        else: print("Incorrect clearance!")

    def draw_Mincowski_sum(simplex_img):
        simplex_pts = np.where(simplex_img != 0)
        simplex_pts = np.array(simplex_pts).T
        for pt in simplex_pts:
            simplex_img = cv2.circle(simplex_img, (pt[1], pt[0]), r + clearance, 1, -1)
        return simplex_img

    def line_eq(p1, p2):
        m = (p1[1] - p2[1]) / (p1[0] - p2[0])
        b = p1[1] - m * p1[0]
        return m, b

    gird_size = float(input("Enter the desired grid (pixel) size of the map: "))

    img = np.zeros((150, 250), np.uint8)

    # Obstacle 1: The rectangle, defined by half-plane method.
    f1 = img.copy()
    for x in range(0, f1.shape[1]):
        if x >= 50:
            f1[:, x] = 1
    f2 = img.copy()
    for x in range(0, f2.shape[1]):
        if x <= 100:
            f2[:, x] = 1
    f3 = img.copy()
    for y in range(0, f3.shape[0]):
        if y <= 150 - 67.5:
            f3[y, :] = 1
    f4 = img.copy()
    for y in range(0, f4.shape[0]):
        if y >= 150 - 112.5:
            f4[y, :] = 1
    f_square = f1 * f2 * f3 * f4

    # Obstacle 2: The ellipse, defined by semi-algebraic method.
    f_ellipse = img.copy()
    for x in range(0, f_ellipse.shape[1]):
        for y in range(0, f_ellipse.shape[0]):
            if ((x - 140) / 15) ** 2 + ((y - 150 + 120) / 6) ** 2 - 1 <= 0:
                f_ellipse[y, x] = 1

    # Obstacle 3: The circle, defined by semi-algebraic method.
    f_circle = img.copy()
    for x in range(0, f_circle.shape[1]):
        for y in range(0, f_circle.shape[0]):
            if (x - 190) ** 2 + (y - 150 + 130) ** 2 - 15 ** 2 <= 0:
                f_circle[y, x] = 1

    # Obstacle 4: The poly, defined by half-plane method.
    p5 = np.array([150, 150 - 15])
    p6 = np.array([173, 150 - 15])
    p7 = np.array([193, 150 - 52])
    p8 = np.array([170, 150 - 90])
    p9 = np.array([163, 150 - 52])
    p10 = np.array([125, 150 - 56])

    f5 = img.copy()
    m5, b5 = line_eq(p10, p5)
    for x in range(0, f5.shape[1]):
        for y in range(0, f5.shape[0]):
            if y - m5 * x - b5 <= 0:
                f5[y, x] = 1

    f6 = img.copy()
    for y in range(0, f6.shape[0]):
        if y <= p5[1]:
            f6[y, :] = 1

    f7 = img.copy()
    m7, b7 = line_eq(p6, p9)
    for x in range(0, f7.shape[1]):
        for y in range(0, f7.shape[0]):
            if y - m7 * x - b7 >= 0:
                f7[y, x] = 1

    f8 = img.copy()
    m8, b8 = line_eq(p9, p10)
    for x in range(0, f8.shape[1]):
        for y in range(0, f8.shape[0]):
            if y - m8 * x - b8 >= 0:
                f8[y, x] = 1

    f_poly1 = f5 * f6 * f7 * f8

    f9 = img.copy()
    m9, b9 = line_eq(p6, p7)
    for x in range(0, f9.shape[1]):
        for y in range(0, f9.shape[0]):
            if y - m9 * x - b9 <= 0:
                f9[y, x] = 1

    f10 = img.copy()
    m10, b10 = line_eq(p7, p8)
    for x in range(0, f10.shape[1]):
        for y in range(0, f10.shape[0]):
            if y - m10 * x - b10 >= 0:
                f10[y, x] = 1

    f11 = img.copy()
    m11, b11 = line_eq(p8, p9)
    for x in range(0, f11.shape[1]):
        for y in range(0, f11.shape[0]):
            if y - m11 * x - b11 >= 0:
                f11[y, x] = 1

    f12 = img.copy()
    m12, b12 = line_eq(p6, p9)
    for x in range(0, f12.shape[1]):
        for y in range(0, f12.shape[0]):
            if y - m12 * x - b12 <= 0:
                f12[y, x] = 1

    f_poly2 = f9 * f10 * f11 * f12
    f_poly = f_poly1 + f_poly2

    f_bound = img.copy()
    if r or clearance != 0:
        for x in range(0, f_bound.shape[1]):
            for y in range(0, f_bound.shape[0]):
                if x ==0 or y == 0 or x == f_bound.shape[1] - 1 or y == f_bound.shape[0] - 1:
                    f_bound[y, x] = 1

    map = f_square + f_ellipse + f_circle + f_poly + f_bound

    map_mincowski = map.copy()
    map_mincowski = draw_Mincowski_sum(map_mincowski)

    map_size = (int(map.shape[1] / gird_size), int(map.shape[0] / gird_size))
    map_mincowski = cv2.resize(map_mincowski, map_size)
    for x in range(0, map_mincowski.shape[1]):
        for y in range(0, map_mincowski.shape[0]):
            if map_mincowski[y, x] != 0: map_mincowski[y, x] = 1

    return map_mincowski