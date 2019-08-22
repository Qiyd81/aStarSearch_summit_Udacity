from helpers import Map, load_map, show_map
from helper import Maps, load_maps, show_maps
from collections import deque

import numpy as np

def next_status(current_state,roads):  # function to get the next status
    return set(roads[current_state])

def path_cost(path):                   # function to calculate the path cost
    cost = 0
    if path != None:
        for i in range(len(path)-1):                 # accumulate the distance between 2 nodes in the path
            vector1 = np.array(nodes[path[i]])       # convert the nodes value to vector so to use np.linalg.norm to calculate the distance
            vector2 = np.array(nodes[path[i+1]])
            cost += np.linalg.norm(vector2 - vector1) # use the np.linalg.norm to calculate the distance between 2 vectors
    return cost
def h_cost(path, goal):                              # help function to calculate the estimated distance between the path's last node to the destination
    vector2 = np.array(nodes[goal])
    vector1 = np.array(nodes[path[-1]])
    h_cost = np.linalg.norm(vector2 - vector1)
    return h_cost
def total_cost(path, goal):                          # help function to calculate the distance of current explored path, together with the estimated distance
    return path_cost(path)+h_cost(path, goal)

def queue_cost(paths, goal):                         # help function to queue the distances of current explored paths to destination
    cost_queue = []                                  # list to store the costs
    node_indexs = []                                 # list to store the node index in the frontiers
    output_indexs = []                               # list to store the sorted node indexes
    for node in paths:
        cost = total_cost(paths[node], goal)         # calculate each path cost
        cost_queue.append(cost)                      # store the costs
        node_indexs.append(node)                     # store the relevant node indexes
    transfer_indexs = np.array(cost_queue).argsort() # transfer list to store the cost index before sorting into the sorted sequence, this is used to get the relevant node indexes
    for i in range(len(transfer_indexs)):            # get the relevant node indexes
        output_indexs.append(node_indexs[transfer_indexs[i]])
    return output_indexs                             # return the sorted node indexes


def shortest_path(M, start, goal):        # function to find the shortest path
    global nodes 
    nodes = M.intersections                       # dict with the nodes index as keys, and coordinates as the values
    global roads 
    roads = M.roads                               # list of list, with outer list as nodes, and inner list as the roads to the node.

    frontiers = set([start])                         # initialize the frontiers
    explored = set([start])                          # set with explored nodes
    paths = {}      #                                # dict to store the paths from start to explored nodes
    paths[start] = [start]

    if start == goal:                                # deal with special case when start is the goal
        paths = {start:[start]}

    else:
        frontiers = next_status(start, roads)        # start loop from the second step
        while goal not in frontiers:                 # loop until find the goal
            if len(frontiers)==0:                    # if none in frontiers, means not find a solution
                print('no solution found')
                break
            for f in frontiers:                      # for each node in the frontiers, get the path of it, here uses trace back to get the path
                if f in paths:                       # if node f already detected before, then do nothing
                    continue
                else:                                # node not detected before
                    next_fs = next_status(f, roads)  # find the nodes connected to it
                    temp_paths = {}                  # temp dict to store the potential paths found
                    temp_path = [f]                  # current node path
                    for pre_f in next_fs:            # loop over the nodes connected to it
                        if pre_f in paths:           # if some nodes in the explored paths, means current node have a path connected back, it should have at least one
                            temp_paths[pre_f] = paths[pre_f] + temp_path    # add one path to the temp paths dict, so that can sort the paths to find the shortest one,
                                                                            # only the shortest one should be kept. the node pre_f is in the explored paths, it need to add
                                                                            # the current node to build the complete path till now.
                    keep_i = queue_cost(temp_paths, f)                      # sort the temp paths

                    paths[f] = paths[keep_i[0]] + [f]                       # keep the shortest path node keep_i[0], and add the path to current node to paths.

            for p in (explored-frontiers):                                  # to sort the paths to current nodes, need to delete the past paths, so if the nodes in explored
                                                                            # but not in frontiers, the paths should be deleted
                if p in paths:                                              # sometimes, some old paths already deleted in previous loop, in that case, do nothing, and only
                                                                            # those not deleted ones, delete it
                    paths.pop(p)
                else:
                    continue

            path_index = queue_cost(paths,goal)                             # sort the paths related to current nodes in frontiers
            selected_node = path_index[0]                                   # select the shortest one
            explored.add(selected_node)                                     # move the shortest one node to explored
            add_nodes = next_status(selected_node, roads)                   # find the next nodes to this shortest path node
            if goal not in add_nodes:                                       # if not reach the goal yet, then update the frontiers with the new nodes
                frontiers = frontiers.union(add_nodes) - explored
            else:                                                           # if reached the goal, then assign the path to the goal
                paths[goal] = paths[selected_node] + [goal]
                frontiers = frontiers.union(add_nodes) - explored           # add the goal to the frontiers, so it will stop at next loop
    print("shortest path called")
    return paths[goal]

