import numpy as np
from collections import deque
import logging
import matplotlib.pyplot as plt

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)

def process_grid(current_grid, ball_color='R', wall_color='D', floor_color='W', target_color='G'):
    cols, rows = len(current_grid[0]), len(current_grid)
    graph = [[0 for _ in range(cols)] for _ in range(rows)]
    ball_pos, ball_cnt = (0, 0), 0
    tar_pos, tar_cnt = (0, 0), 0

    for i in range(rows):
        for j in range(cols):
            # graph[i][j] = 0
            if current_grid[i][j] == ball_color:
                ball_pos = (i + ball_pos[0], j + ball_pos[1])
                ball_cnt += 1
            elif current_grid[i][j] == target_color:
                tar_pos = (i + tar_pos[0], j + tar_pos[1])
                tar_cnt += 1
            elif current_grid[i][j] == wall_color: 
                graph[i][j] = 1

    ball_pos = (ball_pos[0] // ball_cnt, ball_pos[1] // ball_cnt) if ball_cnt > 0 else (0, 0)
    tar_pos = (tar_pos[0] // tar_cnt, tar_pos[1] // tar_cnt) if tar_cnt > 0 else (0, 0)

    return graph, ball_pos, tar_pos

def t_breadth_first_search(graph, grid, source, target, next_level=5):
    if graph[source[0]][source[1]] == 1 or graph[target[0]][target[1]] == 1:
        logging.error("Source or target is a wall.")
        return (-1001, -1001)

    if source == target:
        logging.info("No movement needed.")
        return (0, 0) # Do not move if we are already at the source
    
    directions = 16
    dx = [1, -1, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 2, -2, 2, -2]
    dy = [0, 0, 1, -1, 1, -1, -1, 1, 2, -2, -2, 2, 1, -1, -1, 1]

    find_target = False
    rows = len(graph)
    cols = len(graph[0])
    visited = [[-1 for _ in range(cols)] for _ in range(rows)]

    queue = deque()
    queue.append(source)

    # BFS loop
    while len(queue) > 0:
        current = queue.popleft()
        for i in range(directions):
            if dx[i] + current[0] < 0 or dx[i] + current[0] >= rows or \
                dy[i] + current[1] < 0 or dy[i] + current[1] >= cols:
                continue
            if graph[dx[i] + current[0]][dy[i] + current[1]] == 1 or \
                visited[dx[i] + current[0]][dy[i] + current[1]] != -1:
                continue    
            if (dx[i] + current[0], dy[i] + current[1]) == target:
                find_target = True
                break
            queue.append((dx[i] + current[0], dy[i] + current[1]))
            visited[dx[i] + current[0]][dy[i] + current[1]] = (i ^ 1) # Store the direction from which we came

    if not find_target:
        logging.error("Target not reachable from source.")
        return (-1001, -1001)
        
    # Return the overall direction
    overall_direction = (0, 0)
    prv = None
    current_grid = target
    alpha = [200, 80, 50, 30, 20, 7, 5, 3, 3, 3]

    # for i in range(next_level * 200):
    for i in range(next_level):
        # Not sure if we should stop here
        if current_grid == source:
            break
            # return (0, 0) 
        
        dir = visited[current_grid[0]][current_grid[1]]
        overall_direction = (overall_direction[0] + dx[dir] * alpha[i], overall_direction[1] + dy[dir] * alpha[i])
        current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])
    return ((overall_direction[0]) // 100, (overall_direction[1]) // 100)

    # for i in range(5):
    #     if current_grid == source:
    #         break
        
    #     dir = visited[current_grid[0]][current_grid[1]]
    #     if prv and prv != dir:
    #         return (overall_direction[0] // i + dx[dir], overall_direction[1] // i + dy[dir]) 
    #     else:
    #         prv = dir
    #     overall_direction = (overall_direction[0] + dx[dir], overall_direction[1] + dy[dir])
    #     current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])
    #     print("Current position:", current_grid)

    return ((overall_direction[0]) // 5 * 2, (overall_direction[1]) // 5 * 2)

def breadth_first_search(graph, grid, source, target, next_level=5):
    if graph[source[0]][source[1]] == 1 or graph[target[0]][target[1]] == 1:
        logging.error("Source or target is a wall.")
        return (-1001, -1001)

    if source == target:
        logging.info("No movement needed.")
        return (0, 0)  # Do not move if we are already at the source

    directions = 16
    dx = [1, -1, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 2, -2, 2, -2]
    dy = [0, 0, 1, -1, 1, -1, -1, 1, 2, -2, -2, 2, 1, -1, -1, 1]


    find_target = False
    rows = len(graph)
    cols = len(graph[0])
    visited = [[-1 for _ in range(cols)] for _ in range(rows)]

    queue = deque()
    queue.append(source)
    visited[source[0]][source[1]] = -2  # Mark source visited

    # BFS loop
    while len(queue) > 0:
        current = queue.popleft()
        for i in range(directions):
            ny, nx = current[0] + dx[i], current[1] + dy[i]
            if ny-1 < 0 or ny+1 >= rows or nx-1 < 0 or nx+1 >= cols:
                continue
            if graph[ny][nx] == 1 or visited[ny][nx] != -1:
                continue
            if abs(dx[i]) == 2 and (graph[current[0]+dx[i]//abs(dx[i])][nx] == 1 or visited[current[0]+dx[i]//abs(dx[i])][nx] != -1):
                continue
            if abs(dy[i]) == 2 and (graph[ny][current[1]+dy[i]//abs(dy[i])] == 1 or visited[ny][current[1]+dy[i]//abs(dy[i])] != -1):
                continue
            visited[ny][nx] = i ^ 1  # Store direction from which we came
            if (ny, nx) == target:
                find_target = True
                break
            queue.append((ny, nx))
        if find_target:
            break

    if not find_target:
        logging.error("Target not reachable from source.")
        return (-1001, -1001)

    # Reconstruct path from target to source up to next_level steps
    current_grid = target
    path = []
    overall_direction = (0, 0)
    prv = None

    alpha = [200, 80, 50, 30, 20, 7, 5, 3, 3, 3]

    # for i in range(next_level * 200):
    for i in range(next_level):
        path.append(current_grid)
        # Not sure if we should stop here
        if current_grid == source:
            break
            # return (0, 0) 
        
        dir = visited[current_grid[0]][current_grid[1]]
        overall_direction = (overall_direction[0] + dx[dir] * alpha[i], overall_direction[1] + dy[dir] * alpha[i])
        current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])

    # for i in range(500*next_level):
    #     if current_grid == source:
    #         break
    #     path.append(current_grid)
    #     dir = visited[current_grid[0]][current_grid[1]]
    #     if dir == -1:
    #         break
    #     if prv and prv != dir:
    #         if i < next_level:
    #             overall_direction = (overall_direction[0] // max(i,1) + dx[dir], overall_direction[1] // max(i,1) + dy[dir])
    #         # break
    #         continue
    #     else:
    #         prv = dir
    #     if i < next_level:
    #         overall_direction = (overall_direction[0] + dx[dir], overall_direction[1] + dy[dir])
    #     current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])

    # Visualization part
    import matplotlib.pyplot as plt

    color_map = {
        0: 'white',  # free cell
        1: 'black',  # wall
    }

    fig, ax = plt.subplots()
    for y in range(rows):
        for x in range(cols):
            color = color_map[graph[y][x]]
            if (y, x) == source:
                color = 'green'
            elif (y, x) == target:
                color = 'red'
            elif (y, x) in path:
                color = 'purple'
            rect = plt.Rectangle((x, rows - 1 - y), 1, 1, facecolor=color, edgecolor='gray')
            ax.add_patch(rect)

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect('equal')
    ax.axis('off')
    plt.title("BFS Path Visualization")
    plt.show()

    return (overall_direction[0] // 100, overall_direction[1] // 100)
