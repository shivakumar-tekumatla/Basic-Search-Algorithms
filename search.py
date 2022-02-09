# Basic searching algorithms

import sys
from matplotlib.cbook import violin_stats 

def init(grid):
    """This function intializes all the required variables"""
    graph_nodes = graph(grid)
    queue =[]
    visited=[]
    path = []
    steps = 0
    found = False
    inf = sys.maxsize
    cost_grid ={}
    path_track = {}
    cost =1 
    return graph_nodes,queue,visited,path,steps,found,inf,cost_grid,path_track,cost

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
        if row+1 in range(0,len(grid)) and grid[row+1][col]==0:down = [row+1,col]
        else:down = None 
        if row-1 in range(0,len(grid)) and grid[row-1][col]==0:up = [row-1,col]
        else:up = None 
        if col+1 in range(0,len(grid[row])) and grid[row][col+1]==0:right = [row,col+1]
        else:right = None 
        if col-1 in range(0,len(grid[row])) and grid[row][col-1]==0:left = [row,col-1]
        else:left = None 
        out  = [right,down,left,up]
        return [i for i in out if i is not None ]
    nodes ={}
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            if grid[row][col]==0:
                nodes[(row,col)] = check(row,col,grid)

    return nodes 

def heuristic(node,goal):
    """Manhattan Heuristic function.
    For a given node and goal, this function return manhattan distance between them"""
    x_node,y_node = node 
    x_goal,y_goal = goal
    return abs(x_goal-x_node)+abs(y_goal-y_node)


def shortest_path_finder(start,goal,path_track,found):
    """This function returns the shortest path in the explored nodes"""
    path=[]
    path.append(goal)
    node=goal
    if found:
        while node!=start:
            node = path_track[tuple(node)]
            path.append(node)
    else:
        path=[]
    path.reverse()
    return path

def bfs_dfs_search(grid,start,goal,search_method):
    graph_nodes,queue,visited,path,steps,found,inf,cost_grid,path_track,cost = init(grid)

    visited=set()
    queue.append(start)

    while queue and not found:
        if search_method =="bfs":
            item = queue.pop(0)    # Dequeuing the first element in the queue
            for node in graph_nodes[tuple(item)]:
                if tuple(node) not in visited:
                    visited.add(tuple(node))
                    queue.append(node)
                    path_track[tuple(node)] = item 
                    if node == goal:
                        found = True
                        break
            
        else:
            item = queue.pop()
            idx =-1
            if tuple(item) not in visited:
                visited.add(tuple(item))
                for node in graph_nodes[tuple(item)]:
                    queue.insert(idx,node)
                    if tuple(node) in path_track.keys():

                        path_track[tuple(item)] = node 
                    else:
                        path_track[tuple(node)] = item 
                    idx-=1
                    if node == goal:
                        found = True
                        break
            

    path=shortest_path_finder(start,goal,path_track,found)
    steps = len(visited)
    return path, steps, found
def dijkstra_astar_search(grid,start,goal,search_method):
    """This is a common function for dijkstra and A* . The both search methods are almost same.
    We need use heuristic for A*."""
    graph_nodes,queue,visited,path,steps,found,inf,cost_grid,path_track,cost = init(grid)
    # visited = set()
    for key in graph_nodes.keys():
        cost_grid[key]=inf 

    cost_grid[tuple(start)] = 0
    visited.append(start)
    queue.append(start)
    steps+=1
    step_flag = False
    while queue and not found:
        _min = inf
        for key in queue:
            if search_method == "astar":
                step_flag = True
                if cost_grid[tuple(key)]+ heuristic(key,goal) <_min:
                    _min = cost_grid[tuple(key)]+ heuristic(key,goal)
                    min_key = key
            elif search_method=="dijkstra":
                if cost_grid[tuple(key)] <_min:
                    _min = cost_grid[tuple(key)]
                    min_key = key 
        item= tuple(min_key)
        queue.remove(list(item))
        if step_flag:
            steps+=1
        for node in graph_nodes[item]:
            if node not in visited:
                if cost_grid[tuple(node)]> cost_grid[item]+cost :
                    cost_grid[tuple(node)]= cost_grid[item]+cost 

                visited.append(node)
                queue.append(node)
                path_track[tuple(node)] = list(item) ## Keeping track of the parent 
                if not step_flag:
                    steps+=1
                if node == goal:
                    found = True
                    break 

    path=shortest_path_finder(start,goal,path_track,found)
    return path, steps, found

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
    
    search_method = "bfs"
    path,steps,found = bfs_dfs_search(grid,start,goal,search_method)
    # print(path)
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
    
    search_method = "dfs"
    path,steps,found = bfs_dfs_search(grid,start,goal,search_method)
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
    search_method = "dijkstra"
    path,steps,found = dijkstra_astar_search(grid,start,goal,search_method)

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
    search_method = "astar"
    path,steps,found = dijkstra_astar_search(grid,start,goal,search_method)
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
