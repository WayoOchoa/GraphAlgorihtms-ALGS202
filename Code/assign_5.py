import numpy as np
import time, math
import random
import heapq

class Point2D():
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Vertex():
    def __init__(self,value):
        # For cartesian coordinates the value param is a tuple of pairs (x,y)
        self.value = value
        self.prev_node = []

    def get_value(self):
        return (self.value.x,self.value.y)

class DisjointSets():
    def __init__(self,n):
        self.parents = [i for i in range(n)]
        self.ranks = [1] * n # Heihgt of each subtree
    
    def Find(self,u):
        # If u is a parent of itself then is the root node of the subtree
        if self.parents[u] == u:
            return u
        else:
            # Find the representative node of the subtree
            result =  self.Find(self.parents[u])
            # We cache the result by moving u's node directly under the
            # representative of the set
            self.parents[u] = result

            return result

    def Union(self,v,u):
        # Find the current sets of v and u
        vset = self.Find(v)
        uset = self.Find(u)

        # if they have the same root then they are already part
        # of the same tree
        if vset == uset:
            return
        
        # Put the smaller ranked tree (less height) under the
        # bigger ranked item
        if self.ranks[uset] < self.ranks[vset]:
            self.parents[uset] = vset
        elif self.ranks[vset] < self.ranks[uset]:
            self.parents[vset] = uset
        # if the ranks are the same then put any of them under the
        # other but increase the rank of the tree
        else:
            self.parents[uset] = vset
            self.ranks[vset] = self.ranks[vset] + 1


class Graph():
    def __init__(self, type='undirected'):
        self.graph = dict()
        self.type = type
    
    def AddEdge(self,u,v,weight=1):
        if not self.is_in_graph(u):
            self.graph[Vertex(u)] = []
        if not self.is_in_graph(v):
            self.graph[Vertex(v)] = []

    def AddVertex(self,v):
        if not self.is_in_graph(v):
            self.graph[Vertex(v)] = []

    def is_in_graph(self,node):
        """Function that compares if a Point2D object is already
        a vertex in the graph

        Params:

            node: Point2D(x,y) object"""
        for i in self.graph:
            v = i.get_value()
            if v[0] == node.x and v[1] == node.y:
                return True
        
        return False
    
    def minimum_cost(self):
        cost = dict()
        parent = dict()
        visited = dict()
        for v in self.graph:
            node = v.get_value()
            cost[node] = 10**7
            visited[node] = 0
            parent[node] = -1 #Points to no one
        
        seed = random.choice(list(cost.keys())) #randomize the root selection
        #seed = (0,0)
        cost[seed] = 0

        # Creating Priority Queue
        prioQ = []
        for v in self.graph:
            node = v.get_value()
            heapq.heappush(prioQ,(cost[node],node))

        # Prims algorithm implementation
        while bool(prioQ):
            costv,v = heapq.heappop(prioQ)
            visited[v] = 1 
            for u in self.graph:
                u_value = u.get_value()
                #if any(u_value in i for i in prioQ): #Check if the vertex is inside the PriorityQueue
                if not visited[u_value]:
                    distance_vu = math.sqrt((v[0]-u_value[0])**2 + (v[1]-u_value[1])**2)
                    if cost[u_value] > distance_vu:
                        cost[u_value] = distance_vu
                        parent[u_value] = v
                        heapq.heappush(prioQ,(cost[u_value],u_value))
        
        min_cost = 0
        for d in cost:
            min_cost += cost[d]
        
        return min_cost
    
    def Clustering(self,k):
        n = len(self.graph)
        clusters = DisjointSets(n)
        number_classes = n

        # Giving ids to the nodes in the graph
        ids = dict()
        count = 0
        for i in self.graph:
            j = i.get_value()
            ids[j] = count
            count += 1

        # Computing the edges weights by euclidean distance
        edges = []
        for v in self.graph:
            v_value = v.get_value()
            for u in self.graph:
                u_value = u.get_value()
                if v_value != u_value:
                    distance = math.sqrt((v_value[0]-u_value[0])**2 + (v_value[1]-u_value[1])**2)
                    edges.append((ids[v_value],ids[u_value],distance))
        # Sorting all edges
        edges.sort(key=lambda edge: edge[2])

        max_distance = 10**7 # The max distance that can be used to maintain the number of classes equal to k
        for e in edges:
            if clusters.Find(e[0]) == clusters.Find(e[1]):
                continue
            if number_classes == k and max_distance > e[2]:
                max_distance = e[2]
                continue
            if number_classes > k:
                clusters.Union(e[0],e[1])
                number_classes -= 1

        return max_distance

def main():
    s1 = Graph()
    s1.AddVertex(Point2D(0,0))
    s1.AddVertex(Point2D(0,2))
    s1.AddVertex(Point2D(1,1))
    s1.AddVertex(Point2D(3,0))
    s1.AddVertex(Point2D(3,2))

    #print(s1.minimum_cost())

    s2 = Graph()
    s2.AddVertex(Point2D(7,6))
    s2.AddVertex(Point2D(4,3))
    s2.AddVertex(Point2D(5,1))
    s2.AddVertex(Point2D(1,7))
    s2.AddVertex(Point2D(2,7))
    s2.AddVertex(Point2D(5,7))
    s2.AddVertex(Point2D(3,3))
    s2.AddVertex(Point2D(7,8))
    s2.AddVertex(Point2D(2,8))
    s2.AddVertex(Point2D(4,4))
    s2.AddVertex(Point2D(6,7))
    s2.AddVertex(Point2D(2,6))

    print(s2.Clustering(3))

if __name__=='__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))