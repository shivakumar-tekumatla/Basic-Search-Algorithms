import csv
# from importlib.resources import path

from matplotlib.pyplot import get 
from search import graph,shortest_path_finder 

from main import load_map

grid, start, goal = load_map('test_map.csv')

graph_nodes = graph(grid)


# queue =[]
# visited=[]
# path = []
# steps = 0
# found = False
# cost_grid ={}
path_track = {}
# cost =1 

# queue.append(start)
# visited.append(start)
# steps+=1
# while queue and not found:
#     item = queue.pop()    # Dequeuing the first element in the queue
#     # idx=-1
#     for node in graph_nodes[tuple(item)]:
#         if node not in visited:
#             visited.append(node)
#             queue.append(node)
#             # idx-=1
#             path_track[tuple(node)] = item
#             steps+=1
#             if node == goal:
#                 found = True
#                 break
# print(steps)
# path=shortest_path_finder(start,goal,path_track,found)
# print(path)
# print(visited)
search_method = "dfs"
visited = set()
path =[]
queue = []
found = False 
queue.append(start)
while queue and not found:
    item = queue.pop()
    # print("item",item)
    idx =-1
    if tuple(item) not in visited:
        visited.add(tuple(item))
        # path_track[tuple(item)] = []
        for node in graph_nodes[tuple(item)]:
            print(item,node)
            queue.insert(idx,node)
            if tuple(node) in path_track.keys():

                path_track[tuple(item)] = node 
            else:
                path_track[tuple(node)] = item 
            idx-=1
            
            if node == goal:
                found = True
                break
print(len(visited))
# print(path_track)
for key in path_track:
    print(key,path_track[key])
print(path_track[(3,3)])
path=shortest_path_finder(start,goal,path_track,found)
print(path)
# print("*"*100)
# queue =[]
# visited=set()
# path = []
# found = False

# cost_grid ={}
# path_track = {}
# cost =1 

# queue.append(start)
# # visited.add(start)

# while queue and not found:
#     item = queue.pop(0)    # Dequeuing the first element in the queue
#     for node in graph_nodes[tuple(item)]:
#         if tuple(node) not in visited:
#             visited.add(tuple(node))
#             queue.append(node)
#             path_track[tuple(node)] = item 
#             if node == goal:
#                 found = True
#                 break
# print(len(visited))

# print("*"*100)
# search_method = "dfs"
# visited = set()
# path =[]
# queue = []
# found = False 
# queue.append(start)

# queue.append(start)
# while queue and not found:
#     if search_method=="bfs":
#         item = queue.pop(0)    # Dequeuing the first element in the queue
#         for node in graph_nodes[tuple(item)]:
#             if tuple(node) not in visited:
#                 visited.add(tuple(node))
#                 queue.append(node)
#                 path_track[tuple(node)] = item 
#                 if node == goal:
#                     found = True
#                     break

#     elif search_method =="dfs":
#         item = queue.pop()
#         idx =-1
    
#         if tuple(item) not in visited:
#             visited.add(tuple(item))
        
#         for node in graph_nodes[tuple(item)]:
#             queue.insert(idx,node)
#             path_track[tuple(node)] = item
#             idx-=1
            
#             if node == goal:
#                 found = True
#                 break
# print(len(visited))