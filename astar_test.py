import csv
from email.headerregistry import HeaderRegistry

from matplotlib.pyplot import get 
from search import graph,shortest_path_finder ,heuristic
import sys 
from main import load_map


grid, start, goal = load_map('test_map.csv')

def dijkstra_astar_search(grid,start,goal,search_method):
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
            else:
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
                path_track[tuple(node)] = list(item) 
                if not step_flag:
                    steps+=1
                if node == goal:
                    found = True
                    break 
    path=shortest_path_finder(start,goal,path_track,found)
    return path, steps, found


print(dijkstra_astar_search(grid,start,goal,"astar"))

"""    # search_method = "astar"
    # path,steps,found = dijkstra_astar_search(grid,start,goal,search_method)
    # print(path)
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

    for key in graph_nodes.keys():
        cost_grid[key]=inf 

    cost_grid[tuple(start)] = 0
    visited.append(start)
    queue.append(start)
    steps+=1


    while queue and not found:
        item = tuple(find_min_key(queue,cost_grid,goal))  # finding the node with minimum cost 
        queue.remove(list(item))
        steps+=1
        for node in graph_nodes[item]:
            if node not in visited:
                if cost_grid[tuple(node)]> cost_grid[item]+cost :
                    cost_grid[tuple(node)]= cost_grid[item]+cost 

                visited.append(node)
                queue.append(node)
                path_track[tuple(node)] = list(item) 
                
                if node == goal:
                    found = True
                    break 

    path=shortest_path_finder(start,goal,path_track,found)"""


"""
    def dijkstra_astar_search(grid,start,goal,search_method):
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

    for key in graph_nodes.keys():
        cost_grid[key]=inf
    cost_grid[tuple(start)] = 0
    queue.append(start)
    visited.append(start)
    steps+=1
    while queue and not found:
        item = min(zip(cost_grid.values(), cost_grid.keys()))[1]  # finding the node with minimum cost 
        
        for node in graph_nodes[item]:
            if node not in visited:
                if search_method == "dijkstra":
                    if cost_grid[tuple(node)]> cost_grid[item]+cost :
                        cost_grid[tuple(node)]= cost_grid[item]+cost 

                else:
                    if cost_grid[tuple(node)]> cost_grid[item]+heuristic(node,goal)+cost :
                        cost_grid[tuple(node)]= cost_grid[item]+heuristic(node,goal)+cost 
            
                visited.append(node)
                queue.append(node)
                path_track[tuple(node)] = list(item) 
                # print(node,cost_grid[tuple(node)])
                steps+=1
                
            if node == goal:
                found = True
                # print(queue)
                break 
        del cost_grid[item]
    # print(visited)
    path=shortest_path_finder(start,goal,path_track,found)
    return path, steps, found"""

"""def find_min_key(queue,cost_grid,goal):
    _min = sys.maxsize
    for key in queue:
        if cost_grid[tuple(key)]+ heuristic(key,goal) <_min:
            _min = cost_grid[tuple(key)]+ heuristic(key,goal)
            min_key = key 
    return min_key 
"""