import sys
import numpy as np
import heapq #part of standard lib
import time
def main():
    if len(sys.argv) != 3:
        print("Usage: python3planner planner.py <algorithm> <world-file>")
        sys.exit(1)

    # Parse args
    algorithm = sys.argv[1]
    world = sys.argv[2]
    # Keep track of position and dirty cell locations
    starting_pos = (-1,-1)
    dirty_cells = []
    # Parse text file (world)
    rows, cols, world_arr = parse_world(world)
    # Identify starting position and dirty cells
    for x in range(rows):
        for y in range(cols):
            if world_arr[x][y] == '@':
                starting_pos = (x,y)
            if world_arr[x][y] == '*':
                dirty_cells.append((x,y))
                #print(world_arr[x][y])


    # Choose algorithm: UCS
    if algorithm == "uniform-cost":
        uniform_cost_search(starting_pos, dirty_cells, world_arr, rows, cols)

    # Choose algorithm: DFS
    elif algorithm == "depth-first":
        depth_first_search(starting_pos, dirty_cells, world_arr, rows, cols)

    # Choose algorithm: Invalid
    else:
        print("Invalid algorithm. Use either \"uniform-cost\" or \"depth-first\". Exiting...")
        sys.exit(1)


def parse_world(world_file):
    """Parse the world file into a 2D numpy array.
    Returns the number of rows, columns, and the numpy array."""
    with open(world_file, 'r', encoding="utf-16") as f:
        # Read rows and cols
        cols = int(f.readline().strip())
        rows = int(f.readline().strip())

        # Read grid into numpy array
        world = []
        for x in range(rows):
            data_row = f.readline().strip()
            data_row = list(data_row)
            world.append(data_row)

        np_world = np.array(world)

    return rows, cols, np_world

def uniform_cost_search(starting_pos, dirty_cells, world_arr, rows, cols):
                                 # Create immutable set of dirty cells
    start_state = (starting_pos, frozenset(dirty_cells))
    frontier = []
    heapq.heappush(frontier, (0, start_state, [])) # cost, state, path
    visited = {}

    # Initialize nodes generated/expanded
    nodes_generated = 1
    nodes_expanded = 0

    while frontier:
        cost, (pos, remaining_dirt), path = heapq.heappop(frontier)

        # Reached goal?
        if len(remaining_dirt) == 0:
            print("Vrroooomm")
            for action in path:
                print(action)
            print(f"Nodes generated: {nodes_generated}")
            print(f"Nodes expanded: {nodes_expanded}")
            return
        # Else
        state_id = (pos, remaining_dirt)
        if state_id in visited and visited[state_id] <= cost:
            continue # We've seen this node at a lower/equal cost, skip.

        visited[state_id] = cost
        nodes_expanded += 1

        # Explore N,S,W,E
        for direction, coords in [('N', (-1,0)), ('E', (0,1)), ('S', (1,0)), ('W', (0,-1)) ]:
            new_x = pos[0] + coords[0]
            new_y = pos[1] + coords[1]
            if 0 <= new_y < cols and 0 <= new_x < rows and world_arr[new_x][new_y] != '#':
                new_pos = (new_x, new_y)
                new_state = (new_pos, remaining_dirt)
                heapq.heappush(frontier, (cost + 1, new_state, path + [direction]))
                nodes_generated += 1

        # Vacuum if dirty
        if pos in remaining_dirt:
            # Update new state to account for 1 less dirty cell
            new_dirt = remaining_dirt - {pos}
            new_state = (pos, new_dirt)
            heapq.heappush(frontier, (cost + 1, new_state, path + ['V']))
            nodes_generated += 1


def depth_first_search(starting_pos, dirty_cells, world_arr, rows, cols):
    start_state = (starting_pos, frozenset(dirty_cells))
    stack = []
    stack.append((start_state, [])) # state, path
    visited = set()

    nodes_generated = 1
    nodes_expanded = 0
    print(len(stack))
    while len(stack) > 0:

        (pos, remaining_dirt), path = stack.pop()

        # Reached goal?
        if len(remaining_dirt) == 0:
            print("Vrroooomm")
            for action in path:
                print(action)
            print(f"Nodes generated: {nodes_generated}")
            print(f"Nodes expanded: {nodes_expanded}")
            return

        #Else
        state_id = (pos, remaining_dirt)
        if state_id in visited:
            continue # Already seen this node
        nodes_expanded += 1

        # Add node to visited nodes
        visited.add(state_id)

        # Explore N,W,S,E
        for direction, coords in [('N', (-1, 0)), ('E', (0, 1)), ('S', (1, 0)), ('W', (0, -1))]:
            new_x = pos[0] + coords[0]
            new_y = pos[1] + coords[1]
            if 0 <= new_y < cols and 0 <= new_x < rows and world_arr[new_x][new_y] != '#':
                new_pos = (new_x, new_y)
                new_state = (new_pos, remaining_dirt)
                stack.append((new_state, path + [direction]))
                nodes_generated += 1

        # Vacuum on dirty spot
        if pos in remaining_dirt:
            new_dirt = remaining_dirt - {pos}
            new_state = (pos, new_dirt)
            stack.append((new_state, path + ['V']))
            nodes_generated += 1
if __name__ == "__main__":
    main()