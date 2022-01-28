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