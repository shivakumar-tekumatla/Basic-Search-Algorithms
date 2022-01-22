# Basic searching algorithms
from operator import truediv
from xml.dom import NotFoundErr


def graph(grid):
    """
    Returns the graph for a given grid. 

    This function forms the graph based on given grid.
    At each node, the adjacent nodes are added in [right,down,left,up] order.
    Nodes are added only if the corresponding node is a space not an obstacle.
    That means , if node [2,1] is obstacle , it will not be added to the graph. 
    """
    def check(row,col,grid):
        """This function checks if a corresponding element in grid is a space or an obstacle.
        returns valid nodes in the order [right,down,left,up]
        """
        if row+1 in range(0,len(grid)) and grid[row+1][col]==0:
            down = [row+1,col]
        else:
            down = None 
        if row-1 in range(0,len(grid)) and grid[row-1][col]==0:
            up = [row-1,col]
        else:
            up = None 
        if col+1 in range(0,len(grid[row])) and grid[row][col+1]==0:
            right = [row,col+1]
        else:
            right = None 
        if col-1 in range(0,len(grid[row])) and grid[row][col-1]==0:
            left = [row,col-1]
        else:
            left = None 
        out  = [right,down,left,up]
        return [i for i in out if i is not None ]
    nodes ={}
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            if grid[row][col]==0:
                nodes[(row,col)] = check(row,col,grid)

    return nodes 

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    graph_nodes = graph(grid)
    
    queue =[]
    visited=[]
    path = []
    path_track={}
    steps = 0
    found = False

    queue.append(start)
    visited.append(start)

    while queue or not found:
        item = queue.pop(0)    # Dequeuing the first element in the queue
        if item == goal:
            found = True
            break 
        for node in graph_nodes[tuple(item)]:
            if node not in visited:
                visited.append(node)
                queue.append(node)
                path_track[tuple(node)] = item  # Keeping track of the parents of each node 
                
                if node == goal:
                    found = True
                    break
                else:
                    steps+=1

    # BFS exploration is done. Shortest path in BFS should be computed 
    path.append(goal)
    node=goal
    if found:
        while node!=start:
            node = path_track[tuple(node)]
            path.append(node)
    else:
        path=[]
    
    path.reverse()



    # while not found or  queue:
    #     item = queue.pop(0)
    #     # if item == goal:
    #     #     found = True
    #     #     steps+=1
    #     #     break
    #     for node in graph_nodes[tuple(item)]:
    #         if node not in visited:
    #             visited.append(node)
    #             queue.append(node)
    #             steps+=1
    #             if node == goal:
    #                 found = True
    #                 break


    # path=visited         

    # for item in queue:
    #     if item not in path and not found:
    #         path.append(item)
    #         steps+=1
    #         for val in graph_nodes[tuple(item)]:
    #             queue.append(val)
    #         if item ==goal:
    #             found = True
    #             break

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
