import numpy as np
import time

class Graph():
    def __init__(self,type='undirected'):
        self.graph = dict()
        self.type = type
    
    def addEdge(self,node1,node2,weight=1):
        if node1 not in self.graph:
            self.graph[node1] = []
        if node2 not in self.graph:
            self.graph[node2] = []
        
        if self.type == 'undirected':
            self.graph[node1].append(node2)
            self.graph[node2].append(node1)
            #self.graph[node1].append((node2,weight)) # weighted graph
        elif self.type == 'directed':
            self.graph[node1].append(node2)
    
    def BFS(self,startNode):
        vertices = []
        for k in self.graph.keys():
            vertices.append(k)
        
        # initializing
        dist = {} # distance of vertex u from startNode
        d = 0 # distance assigning variable
        for u in vertices:
            dist[u] = -1 # -1 represent infinite distance
        queue = [startNode]
        dist[startNode] = d

        # exploring the graph
        while len(queue) != 0:
            u = queue.pop(0)
            for e in self.graph[u]:
                if dist[e] == -1:
                    queue.append(e)
                    dist[e] = d + 1
            d = d+1
        
        print(dist)
        return dist
    
    def CheckBipartite(self,startNode):
        vertices = []
        for k in self.graph.keys():
            vertices.append(k)
        
        # initializing
        color = {}
        c = 'white'
        for u in vertices:
            color[u] = []
        queue = [startNode]
        color[startNode] = c

        # exploring the graph
        while len(queue) != 0:
            if c == 'white':
                c = 'black'
            elif c == 'black':
                c = 'white'

            u = queue.pop(0)
            for e in self.graph[u]:
                if 'white' not in color[e] and 'black' not in color[e]:
                    queue.append(e)
                    color[e] = c
        
        # check if nodes of one color are only connected to nodes of the other color
        fbipartite = 0
        bipartite_flag = 1
        for v in self.graph:
            same_color_flags = []
            vertex_color = color[v]
            for e in self.graph[v]:
                if vertex_color == color[e]:
                    same_color_flags.append(1)
                else:
                    same_color_flags.append(0)

                if all(num == 0 for num in same_color_flags) and len(same_color_flags) > 0:
                    fbipartite = 1
                else:
                    fbipartite = 0
                
                if fbipartite == 0:  
                    bipartite_flag = 0
        print(bipartite_flag)
        print(color)


def main():
    # Graph object
    ex1 = Graph()
    ex1.addEdge(1,2)
    ex1.addEdge(4,1)
    ex1.addEdge(2,3)
    ex1.addEdge(3,1)

    ex2 = Graph()
    ex2.addEdge(5,2)
    ex2.addEdge(1,3)
    ex2.addEdge(3,4)
    ex2.addEdge(1,4)

    ex3 = Graph()
    ex3.addEdge(5,2)
    ex3.addEdge(4,2)
    ex3.addEdge(3,4)
    ex3.addEdge(1,4)

    #print(ex1.graph)
    ex3.CheckBipartite(4)





if __name__=='__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))