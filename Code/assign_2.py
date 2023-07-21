import numpy as np
import time

def Explore(graph,v,visited):
    visited[v] = 1
    for edge in graph[v]:
        if visited[edge] == 0:
            visited = Explore(graph,edge,visited)
    return visited

def ExploreDirectedGraph(graph,v,visited,explored):
    visited[v] = 1
    explored.append(v)
    fcycle = 0
    # Check neighbors using edge connections
    for edge in graph[v]:
        if edge in explored:
            fcycle = 1
            return visited, fcycle
        if visited[edge] == 0:
            visited, fcycle = ExploreDirectedGraph(graph,edge,visited,explored)
    explored.remove(v)
    return visited, fcycle

def ExploreOrdering(graph,v,visited,visit_orderings,clock):
    visited[v] = 1
    # Assign pre-visit number
    visit_orderings,clock = previsit(v,visit_orderings,clock)
    # follow path
    for edge in graph[v]:
        if visited[edge] == 0:
            visited,visit_orderings,clock = ExploreOrdering(graph,edge,visited,visit_orderings,clock)
    # Assign post-visit number
    visit_orderings,clock = postvisit(v,visit_orderings,clock)
    return visited,visit_orderings,clock

def previsit(v,visit_orderings,clock):
    visit_orderings[v][0] = clock
    clock = clock + 1
    return visit_orderings,clock

def postvisit(v,visit_orderings,clock):
    visit_orderings[v][1] = clock
    clock = clock + 1
    return visit_orderings,clock

def DetectCycles(graph):
    visited = {}
    fcycle = 0
    gflag_cycle = 0
    for i in graph:
        visited[i] = 0
    # Exploring the graph
    for v in graph:
        # follow vertex path
        explored_vertices = []
        if visited[v] == 0:
            visited, fcycle = ExploreDirectedGraph(graph,v,visited,explored_vertices)
        if fcycle == 1:
            gflag_cycle = 1
    return gflag_cycle

def DFS(graph):
    visited = {}
    visit_orderings = {}
    clock = 1
    for i in graph:
        visited[i] = 0
        visit_orderings[i] = [0,0]

    # Explore the graph
    for v in graph:
        if visited[v] == 0:
            visited,visit_orderings,clock = ExploreOrdering(graph,v,visited,visit_orderings,clock)
    
    return visit_orderings

def ReverseGraph(graph):
    # Create reverse graph vertices
    graphR = {}
    for i in graph:
        graphR[i] = []

    # Compute reverse edges
    for v in graph:
        for e in graph[v]:
            graphR[e].append(v)
    return graphR

def SCCs(graph,ordering):
    visited = {}
    for i in graph:
        visited[i] = 0
    
    # Extract order to visit the graph
    ordered_vertices = np.zeros(len(ordering),dtype=int)
    post_num = np.zeros(len(ordering))
    j = 0
    for i in ordering:
        ordered_vertices[j] = i
        post_num[j] = ordering[i][1]
        j = j + 1
    reverse_order = np.flip(ordered_vertices[np.argsort(post_num)])

    # Extract SCCs
    cc = 0
    for v in reverse_order:
        if visited[str(int(v))] == 0:
            cc = cc + 1
            visited = Explore(graph,str(v),visited)
    
    return cc

def main():
    ftop_sort = False
    fSCC = True
    graph = {'1':[],
             '2':['1'],
             '3':['1','2'],
             '4':['1','3'],
             '5':['2','3']}
    
    graphR = ReverseGraph(graph)

    # Detect if there are any cycles in the graph
    flag_cycle = DetectCycles(graphR)
    # Perform DFS
    if not flag_cycle:
        ordering = DFS(graphR)

        # Topological sort
        if ftop_sort:
            ordered_vertices = np.zeros(len(ordering))
            post_num = np.zeros(len(ordering))
            j = 0
            for i in ordering:
                ordered_vertices[j] = i
                post_num[j] = ordering[i][1]
                j = j + 1
            top_sort = np.flip(ordered_vertices[np.argsort(post_num)])
            print(top_sort)

        # Compute SCCs
        if fSCC:
            cc = SCCs(graph,ordering)
            print(cc)

if __name__=='__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))