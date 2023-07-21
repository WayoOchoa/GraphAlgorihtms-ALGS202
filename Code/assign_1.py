import numpy as np
import time

graph = {'1':['2','4'],
         '2':['1','3'],
         '3':['2','4'],
         '4':['1','3']}

def Explore(v,visited,CCnum,connections=0):
    visited[v] = 1
    CCnum[v] = connections
    # Check neighbors using edge connections
    for edge in graph[v]:
        if visited[edge] == 0:
            visited,CCnum = Explore(edge,visited,CCnum,connections)
    return visited,CCnum


def CheckPath(u,v):
    visited = {}
    CCnum = {} # which connected component the vertex belong to
    for i in graph:
        visited[i] = 0
        CCnum[i] = 0
    # Exploring u
    visited,_ = Explore(u,visited,CCnum)

    if visited[v] == 1:
        return 1
    else:
        return 0

def CComponents():
    visited = {}
    CCnum = {} # which connected component the vertex belong to
    for i in graph:
        visited[i] = 0
        CCnum[i] = 0
    # Check number of connections given a graph
    connections = 1
    for i in graph:
        if visited[i] == 0:
            visited,CCnum = Explore(i,visited,CCnum,connections)
            connections = connections + 1 # as it is it will always increase one value more when the algorithm finishes
    return connections-1


def main():
    u = '1'
    v = '4'
    mode = {1:'reachable', 2:'CC'}
    
    task = mode[1]
    if task == 'reachable':
        bpath = CheckPath(u,v)
        print(bpath)
    elif task == 'CC':
        components = CComponents()
        print('The number of connected components is %s' % components)
    else:
        print('Select an available task')

if __name__=='__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))