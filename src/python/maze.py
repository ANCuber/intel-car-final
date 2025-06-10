import numpy as np
from collections import deque

def process_grid(current_grid, ball_color='R', wall_color='D', floor_color='W', target_color='G'):
    cols, rows = len(current_grid[0]), len(current_grid)
    graph = [[1 for _ in range(cols)] for _ in range(rows)]
    ball_pos, ball_cnt = (0, 0), 0
    tar_pos, tar_cnt = (0, 0), 0

    for i in range(rows):
        for j in range(cols):
            graph[i][j] = 0
            if current_grid[i][j] == ball_color:
                ball_pos = (i + ball_pos[0], j + ball_pos[1])
                ball_cnt += 1
            elif current_grid[i][j] == target_color:
                tar_pos = (i + tar_pos[0], j + tar_pos[1])
                tar_cnt += 1
            elif current_grid[i][j] != floor_color: 
                graph[i][j] = 1

    ball_pos = (ball_pos[0] // ball_cnt, ball_pos[1] // ball_cnt) if ball_cnt > 0 else None
    tar_pos = (tar_pos[0] // tar_cnt, tar_pos[1] // tar_cnt) if tar_cnt > 0 else None

    return graph, ball_pos, tar_pos

def breadth_first_search(graph, source, target, next_level=5):
    if graph[source[0]][source[1]] == 1 or graph[target[0]][target[1]] == 1:
        return (0, 0)
        # raise ValueError("Source or target is not reachable.")

    if source == target:
        return (0, 0) # Do not move if we are already at the source
    
    directions = 8
    dx = [1, -1, 0, 0, 1, -1, 1, -1]
    dy = [0, 0, 1, -1, 1, -1, -1, 1]

    find_target = False
    number_of_vertex = len(graph)
    visited = [[-1 for _ in range(number_of_vertex)] for _ in range(number_of_vertex)]

    queue = deque()
    queue.append(source)

    # BFS loop
    while len(queue) > 0:
        current = queue.popleft()
        for i in range(directions):
            if dx[i] + current[0] < 0 or dx[i] + current[0] >= number_of_vertex or \
                dy[i] + current[1] < 0 or dy[i] + current[1] >= number_of_vertex:
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
        return (0, 0)
        # raise ValueError("Target is not reachable from source.")
        
    # Return the overall direction
    overall_direction = (0, 0)
    current_grid = target

    for i in range(next_level):
        # Not sure if we should stop here
        if current_grid == source:
            return (0, 0) 
        
        dir = visited[current_grid[0]][current_grid[1]]
        overall_direction = (overall_direction[0] + dx[dir], overall_direction[1] + dy[dir])
        current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])

    return overall_direction