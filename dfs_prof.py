### YOUR CODE HERE ###
    # graph_nodes = graph(grid)
    
    # queue =[]
    # visited=[]
    # path = []
    # steps = 0
    # found = False
    # path_track={}

    # queue.append(start)
    # visited.append(start)
    # while queue and not found:
    #     item = queue.pop(0)    # Dequeuing the first element in the queue
    #     # path.append(item)
    #     # print(item)
    #     if item == goal:
    #         found = True
    #         break 
    #     if item not in visited:
    #         visited.append(item)
    #     idx=0

    #     for node in graph_nodes[tuple(item)]:
    #         if node not in visited:
    #             visited.append(node)
    #             queue.insert(idx,node)
    #             path_track[tuple(node)] = item
    #             idx+=1
                
    #             if node == goal:
    #                 path.append(goal)
    #                 found = True
    #                 break
    #             else:
    #                 steps+=1
import csv

from matplotlib.pyplot import get 
from search import graph,shortest_path_finder 

from main import load_map

grid, start, goal = load_map('test_map.csv')

graph_nodes = graph(grid)
# for i in graph_nodes:
#     print(i,graph_nodes[i])

def get_parent(nodes,child):
    keys = [k for k, v in nodes.items() if child in v]
    if keys:
        return list(keys[-1])
    return None
    
queue =[]
visited=[]
path = []
path_track={}  # Dictionary to keep track of the path so the optimal path can be constructed later 
steps = 0
found = False
k=[]

queue.append(start)
visited.append(start)

while not found and queue:

    item = queue.pop()
    # k.append(item)
    idx = 0
    # path.append(item)
    for node in graph_nodes[tuple(item)]:
        print(node)
        if node not in visited:
            visited.append(item)
            steps+=1
            queue.insert(idx,node)
            # print(visited,node)
            path_track[tuple(node)] = get_parent(graph_nodes,node)
            idx+=1
        if node==goal:
            found = True
            path_track[tuple(goal)] = get_parent(graph_nodes,goal)
            child =goal
            keys = [k for k, v in graph_nodes.items() if child in v]
            print(keys)
            break 


if found:
        print(f"It takes {steps} steps to find a path using DFS")
else:
    print("No path found")

print(path)

child =[3,3]
keys = [k for k, v in graph_nodes.items() if child in v]
print(keys)

