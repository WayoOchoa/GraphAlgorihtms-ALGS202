import numpy as np
import time
from queue import PriorityQueue, Queue

class Graph():
    def __init__(self,type='undirected'):
        self.graph = dict()
        self.type = type

    def addEgde(self,node1,node2,weight = 1):
        if node1 not in self.graph:
            self.graph[node1] = []
        if node2 not in self.graph:
            self.graph[node2] = []
        
        if self.type == 'undirected':
            self.graph[node1].append((node2,weight))
            self.graph[node2].append((node1,weight))
        elif self.type == 'directed':
            self.graph[node1].append((node2,weight))
    
    def addVertex(self,v):
        if v not in self.graph:
            self.graph[v] = []

    def ShortestPathDijkstra(self,S,nodeu):
        """Computes the shortest path from a starting node S to a end
        node u"""
        # Algorithm initialization
        node_distances = dict()
        prev_node = dict()
        for v in self.graph:
            node_distances[v] = 10**7 # set distances to infinity (represented with -1)
            prev_node[v] = [] # empty idx to prev node
        node_distances[S] = 0

        # Creating the priority queue
        H = PriorityQueue()
        for v in self.graph:
            H.put((node_distances[v],v)) #Put vertex in queue in the format (dist[v],V)
        

        # Exploring graph
        while not H.empty():
            distu,u = H.get()
            for e,w in self.graph[u]:
                if node_distances[e] > distu + w:
                    node_distances[e] = distu + w
                    prev_node[e] = u
                    H.put((node_distances[e],e))
        
        for v in node_distances:
            if node_distances[v] == 10**7:
                node_distances[v] = -1
        
        print(node_distances)
        print(prev_node)
        return node_distances
    
    def BellmanFord(self,S):
        # Algorithm initialization
        node_distances = dict()
        prev_node = dict()
        for v in self.graph:
            node_distances[v] = 10**7 # set distances to infinity (represented with -1)
            prev_node[v] = [] # empty idx to prev node
        node_distances[S] = 0

        for i in range(len(self.graph)):
            for v in self.graph:
                for e,w in self.graph[v]:
                    if node_distances[e] > node_distances[v] + w:
                        if i == len(self.graph) - 1:
                            print(node_distances)
                            return 1
                        node_distances[e] = node_distances[v] + w
                        prev_node[e] = v
        print(node_distances)
        return 0
    
    def BellmanFordShortest(self,s):
        # Initialization
        node_distances = dict()
        prev_node = dict()
        reachable_nodes = dict()
        for v in self.graph:
            node_distances[v] = 10**7
            prev_node[v] = []
            reachable_nodes[v] = 0 # Set to zero if the node cannot be reached
        node_distances[s] = 0
        reachable_nodes[s] = 1

        q = Queue() # Stores the nodes that are reachable by a negative weight cycle

        # Find the shortest path
        for i in range(len(self.graph)):
            for v in self.graph:
                for e,w in self.graph[v]:
                    if node_distances[e] > node_distances[v] + w:
                        if i == len(self.graph) - 1:
                            q.put(e)
                        node_distances[e] = node_distances[v] + w
                        prev_node[e] = v
                        reachable_nodes[e] = 1
        
        # Find negative cycles
        negative_cycle = self.BFS(q)

        for l in negative_cycle:
            if negative_cycle[l]:
                node_distances[l] = -1

        return node_distances
    
    def BFS(self,q):
        negative_cycle = dict()
        for v in self.graph:
            negative_cycle[v] = 0
        
        # Exploring the nodes in queue
        while not q.empty():
            u = q.get()
            negative_cycle[u] = 1
            for e,w in self.graph[u]:
                if not negative_cycle[e]:
                    q.put(e)

        return negative_cycle




def main():
    flight_graph = Graph(type='directed')

    # Edges
    flight_graph.addEgde(1,2,1)
    flight_graph.addEgde(2,3,2)
    flight_graph.addEgde(4,1,2)
    flight_graph.addEgde(3,1,-5)
    flight_graph.addVertex(5)

    distances = flight_graph.BellmanFordShortest(4)

    for v in distances:
        if distances[v] == -1:
            print('-')
        elif distances[v] == 10**7:
            print('*')
        else:
            print(distances[v])

if __name__=='__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))