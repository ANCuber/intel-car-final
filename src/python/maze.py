import numpy as np
from collections import deque
import logging
import cv2

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
                for ii in range(-0, 1):
                    for jj in range(-0, 1):
                        if i+ii < 0 or i+ii >= rows or j+jj < 0 or j+jj >= cols:
                            continue
                        graph[i + ii][j + jj] = 1

    ball_pos = (ball_pos[0] // ball_cnt, ball_pos[1] // ball_cnt) if ball_cnt > 0 else (0, 0)
    tar_pos = (tar_pos[0] // tar_cnt, tar_pos[1] // tar_cnt) if tar_cnt > 0 else (0, 0)

    return graph, ball_pos, tar_pos

def breadth_first_search(graph, grid, source, target, next_level=12):
    if graph[source[0]][source[1]] == 1 or graph[target[0]][target[1]] == 1:
        logging.error("Source or target is a wall.")
        return None, (-1001, -1001)

    if source == target:
        logging.info("No movement needed.")
        return [source], (0, 0)  # No movement needed, return path with source

    directions = 8 # 16
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
        # for i in range(directions - 1, -1, -1):
        for i in range(directions):
            ny, nx = current[0] + dx[i], current[1] + dy[i]
            if ny < 0 or ny >= rows or nx < 0 or nx >= cols:
                continue
            if graph[ny][nx] == 1 or visited[ny][nx] != -1:
                continue
            # if abs(dx[i]) == 2 and (graph[current[0]+dx[i]//abs(dx[i])][nx] == 1 or visited[current[0]+dx[i]//abs(dx[i])][nx] != -1):
            #     continue
            # if abs(dy[i]) == 2 and (graph[ny][current[1]+dy[i]//abs(dy[i])] == 1 or visited[ny][current[1]+dy[i]//abs(dy[i])] != -1):
            #     continue
            visited[ny][nx] = i ^ 1  # Store direction from which we came
            if (ny, nx) == target:
                find_target = True
                break
            queue.append((ny, nx))
        if find_target:
            break

    if not find_target:
        logging.error("Target not reachable from source.")
        return None, (-1001, -1001)

    # Reconstruct path from target to source up to next_level steps
    current_grid = target
    path = []
    overall_direction = (0, 0)
    
    # alpha = [200, 80, 50, 30, 20, 10, 10, 10, 3, 3]
    alpha = np.array([(next_level - i) ** 2 + i for i in range(next_level)])
    sum = np.sum(alpha) / 2

    for i in range(next_level):
        path.append(current_grid)
        # Stop if we reached source
        if current_grid == source:
            break
        
        dir = visited[current_grid[0]][current_grid[1]]
        overall_direction = (overall_direction[0] + dx[dir] * alpha[i], overall_direction[1] + dy[dir] * alpha[i])
        current_grid = (current_grid[0] + dx[dir], current_grid[1] + dy[dir])

    # Add source to path if we haven't reached it yet
    if current_grid != source and source not in path:
        path.append(source)

    # Reverse path to go from source to target (more intuitive for visualization)
    path.reverse()

    return path, (overall_direction[0] // sum, overall_direction[1] // sum)

def visualize_path_on_frame(frame, path, cell_h, cell_w):
    """Draw the BFS path on the frame"""
    if not path:
        return frame
        
    # Draw the path
    for i in range(len(path)-1):
        start_y = path[i][0] * cell_h + cell_h // 2
        start_x = path[i][1] * cell_w + cell_w // 2
        end_y = path[i+1][0] * cell_h + cell_h // 2
        end_x = path[i+1][1] * cell_w + cell_w // 2
        
        # Draw path line (yellow)
        cv2.line(frame, (start_x, start_y), (end_x, end_y), (0, 255, 255), 2)
        
        # Draw dots at each point (blue)
        cv2.circle(frame, (start_x, start_y), 3, (255, 0, 0), -1)
    
    # Draw last point
    if path:
        last_y = path[-1][0] * cell_h + cell_h // 2
        last_x = path[-1][1] * cell_w + cell_w // 2
        cv2.circle(frame, (last_x, last_y), 3, (255, 0, 0), -1)
    
    return frame
