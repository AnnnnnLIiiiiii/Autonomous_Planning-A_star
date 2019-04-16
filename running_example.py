import cv2
import numpy as np
from generate_map import generate_map
from A_star import A_star


def _updateMap(y, x, color, map):
    map[y, x] = color
    cv2.namedWindow("Configuration Space and corresponding path", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Configuration Space and corresponding path", 1000, 600)
    cv2.imshow("Configuration Space and corresponding path", map)
    map[y_start, x_start] = [0, 255, 0]
    map[y_goal, x_goal] = [0, 0, 255]
    cv2.waitKey(1)

map = generate_map()
start_point, goal_point, dij_dict, _map_to_show = A_star(map)
x_start, y_start = int(start_point.split(",")[1]), int(start_point.split(",")[0])
x_goal, y_goal = int(goal_point.split(",")[1]), int(goal_point.split(",")[0])
if goal_point in dij_dict:
    parent = dij_dict[goal_point][1]
    path = [goal_point]
    while parent != start_point:
        child = parent
        parent = dij_dict[child][1]
        path.append(child)
    path.append(start_point)

    path_show = path.copy()

    path_show = [ele for ele in reversed(path_show)]

    for node in path_show:
        y, x = int(node.split(",")[0]), int(node.split(",")[1])
        _updateMap(y, x, [0, 255, 255], _map_to_show)

    print("The path contains following point: ")
    print(path_show)
    cv2.namedWindow("Configuration Space and corresponding path", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Configuration Space and corresponding path", 1000, 600)
    cv2.imshow("Configuration Space and corresponding path", _map_to_show)
    cv2.waitKey()
else:
    print("!!! Cannot find a feasible path !!!")
    cv2.namedWindow("Configuration Space and corresponding path", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Configuration Space and corresponding path", 1000, 600)
    cv2.imshow("Configuration Space and corresponding path", _map_to_show)
    cv2.waitKey()