import heapq
import time

# Representasi grid 2D
grid_str = [
    "S..#......",
    ".#.#.####.",
    ".#......#.",
    ".#####..#.",
    ".....#..#G",
    "####.#..##",
    "...#.#....",
    ".#.#.####.",
    ".#........",
    "....#####."
]

# Parsing grid menjadi array
grid = [list(row) for row in grid_str]
rows, cols = len(grid), len(grid[0])

# Cari koordinat start (S) dan goal (G)
def find_pos(char):
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == char:
                return (i, j)
    return None

start = find_pos('S')
goal = find_pos('G')

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(pos):
    x, y = pos
    neighbors = []
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != '#':
            neighbors.append((nx, ny))
    return neighbors

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def greedy_bfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan(start, goal), start))
    came_from = {}
    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        visited.add(current)

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                heapq.heappush(open_set, (manhattan(neighbor, goal), neighbor))
                if neighbor not in came_from:
                    came_from[neighbor] = current

    return []

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + manhattan(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        visited.add(current)

        for neighbor in get_neighbors(current):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current

    return []

def run_and_display(algorithm, name):
    start_time = time.time()
    path = algorithm(start, goal)
    end_time = time.time()
    time_taken = (end_time - start_time) * 1000  # in ms

    grid_copy = [row[:] for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in ('S', 'G'):
            grid_copy[x][y] = '*'

    print(f"\n{name} Result:")
    for row in grid_copy:
        print(''.join(row))
    print(f"Path length: {len(path)}")
    print(f"Time taken: {time_taken:.2f} ms")

# Jalankan algoritma
run_and_display(greedy_bfs, "Greedy Best-First Search")
run_and_display(a_star, "A* Search")
