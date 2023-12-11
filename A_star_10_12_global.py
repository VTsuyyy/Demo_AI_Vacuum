import heapq

def heuristic(a, b):
    """ Tính toán khoảng cách Manhattan giữa hai điểm a và b """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(matrix, start, goal):
    """ Thuật toán A* """
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + 1

            if 0 <= neighbor[0] < len(matrix) and 0 <= neighbor[1] < len(matrix[0]) and matrix[neighbor[0]][neighbor[1]] != 1:
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return []

def tsp_greedy(matrix, start, goals):
    """ Giải bài toán TSP sử dụng thuật toán Greedy """
    path = [start]
    current_position = start
    to_visit = set(goals)

    while to_visit:
        nearest = None
        nearest_distance = float('inf')

        for goal in to_visit:
            distance = a_star_search(matrix, current_position, goal)
            if distance < nearest_distance and goal != current_position:
                nearest = goal
                nearest_distance = distance

        if nearest:
            path.append(nearest)
            current_position = nearest
            to_visit.remove(nearest)
        else:
            break

    return path


# Sử dụng:
matrix = [[0, 0, 0, 0, 0],
          [0, 1, 1, 1, 0],
          [0, 0, 0, 0, 0],
          [0, 1, 1, 1, 0],
          [0, 0, 0, 0, 0]]

start = (0, 0)
goals = [(4, 4), (2, 2), (3, 3)]

tsp_path = tsp_greedy(matrix, start, goals)
print("TSP Path:", tsp_path)
