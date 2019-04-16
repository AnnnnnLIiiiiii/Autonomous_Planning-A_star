import cv2
import numpy as np

def A_star(_map):
    def updateMap(y, x, color):
        map_to_show[y_start, x_start] = [0, 255, 0]
        map_to_show[y_goal, x_goal] = [0, 0, 255]
        map_to_show[y, x] = color
        cv2.namedWindow("Configuration Space and corresponding path", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Configuration Space and corresponding path", 1000, 600)
        cv2.imshow("Configuration Space and corresponding path", map_to_show)
        cv2.waitKey(1)

    '''Generate the map to plan'''
    map_used = _map
    print("Combine above inputs, the resolution of obstacle space is")
    print(map_used.shape)

    '''Generate the GUI map'''
    map_to_show = map_used.copy()
    for x in range(0, map_to_show.shape[1]):
        for y in range(0, map_to_show.shape[0]):
            if map_to_show[y, x] != 0: map_to_show[y, x] = 255
    map_to_show = cv2.cvtColor(map_to_show, cv2.COLOR_GRAY2BGR)

    ''' Set start and goal node'''
    while 1:
        start_pt = input("Enter the coordinate of start point, separated by a comma: ")
        x_start, y_start = int(start_pt.split(",")[1]), int(start_pt.split(",")[0])
        if x_start in range(0, map_used.shape[1]) and y_start in range(0, map_used.shape[0]):
            if map_used[y_start, x_start] != 1:
                break
            else: print("Cannot start within a obstacles!")
        else: print("Star point not in the map!")
    while 1:
        goal_pt = input("Enter the coordinate of goal point, separated by a comma: ")
        x_goal, y_goal = int(goal_pt.split(",")[1]), int(goal_pt.split(",")[0])
        if x_goal in range(0, map_used.shape[1]) and y_goal in range(0, map_used.shape[0]):
            if map_used[y_goal, x_goal] != 1:
                break
            else: print("Cannot go into a obstacles!")
        else: print("Goal point not in the map!")
    updateMap(y_start, x_start, [0, 255, 255])
    updateMap(y_goal, x_goal, [0, 255, 255])

    def neighbors_with_cost(node):
        neighborS = []
        y, x = int(node.split(",")[0]), int(node.split(",")[1])
        for x_alt in [-1, 0, 1]:
            for y_alt in [-1, 0, 1]:
                x_check, y_check = x + x_alt, y + y_alt
                if x_check in range(0, map_used.shape[1]) and y_check in range(0, map_used.shape[0]) and map_used[y_check, x_check] != 1:
                    '''Don't go out of the map range and avoid obstacles' points'''
                    if abs(x_alt) + abs(y_alt) != 0:
                        if abs(x_alt) + abs(y_alt) == 2:
                            come_update = 2 ** 0.5  # A diagonal neighbor, its cost is sqrt(2) + cost of current node
                        else:
                            come_update = 1  # A vertical or horizontal neighbor, its cost is 1 + cost of current node
                        cost_to_go = ((x_goal - x_check) ** 2 + (y_goal - y_check) ** 2) ** 0.5
                        neighborS.append([y_check, x_check, come_update, cost_to_go])
        return neighborS

    def add_Q(priority, c_come, task):
        entry = [priority, c_come, task]
        if len(pq) == 0:
            pq.append(entry)
        else:
            for idx in range(len(pq)):
                '''
                if 1: Loop through the Q and insert the entry at where the next entry has a bigger cost and previous entry has
                lower or equal cost
                '''
                if pq[idx][0] > priority:
                    pq.insert(idx, entry)
                    break
                if idx == len(pq) - 1:
                    'if there is no such a position, append the entry'
                    pq.append(entry)

    'Initializing'
    cost_to_come = 0
    cost_to_go = ((x_start - x_goal) ** 2 + (y_start - y_goal) ** 2) ** 0.5
    total_cost = cost_to_come + cost_to_go
    pq = [[total_cost, cost_to_come, start_pt]]
    node_dict = {start_pt: [total_cost, None]}
    explored = {start_pt}

    while pq:
        # print('------Q before poping -----')
        # print(len(pq))
        # print(pq)
        v = pq.pop(0)
        cost_to_come = v[1]
        center = v[2]
        # print('------center------')
        # print(center)
        # print('------explored-----')
        # print(explored)
        y, x = int(center.split(",")[0]), int(center.split(",")[1])
        # updateMap(y, x, [0, 255, 0])
        # print('-----frontier------')
        neighbors = neighbors_with_cost(center)
        for neighbor in neighbors:
            node = ",".join([str(neighbor[0]), str(neighbor[1])])
            come_update = neighbor[2]
            cost_to_go = neighbor[3]
            if node not in explored:
                '''
                If the neighbor node is not explored:'
                1. mark this node as explored
                2. insert the node into pq; add_Q will handle the invariant of priority
                3. add the information of the node, its parent and its cost-to-come into node_dict
                '''
                # print(node, cost_to_come + come_update, cost_to_go)
                updateMap(neighbor[0], neighbor[1], [255, 0, 0])
                explored.add(node)
                add_Q(cost_to_come + come_update + cost_to_go, cost_to_come + come_update, node)
                node_dict[node] = [cost_to_come + come_update + cost_to_go, center]
            else:
                '''
                If the neighbor node is explored and current exploration has less cost than previous exploration:'
                1. add this node into priority Q with new cost; the item that has same node but higher cost will still
                   be pop, yet it won't go into the condition.
                2. Update the information of the node in node_dict
                '''
                if node_dict[node][0] > cost_to_come + come_update + cost_to_go:
                    for item in pq:
                        if item[2] == node: del item
                    add_Q(cost_to_come + come_update + cost_to_go, cost_to_come + come_update, node)
                    node_dict[node] = [cost_to_come + come_update + cost_to_go, center]
            if node == goal_pt:
                'If the neighbor node hits the goal node, stop searching and retrun data for path drawing'
                return start_pt, goal_pt, node_dict, map_to_show
    return start_pt, goal_pt, node_dict, map_to_show

