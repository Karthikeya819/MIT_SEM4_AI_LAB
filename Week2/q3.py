# Implement the following undirected weighted graph using class, methods, and data structures of Python.
# Print the adjacency list and adjacency matrix.
class Graph:
    def __init__(self,length):
        self.Vertices = []
        for i in range(length):
            self.Vertices.append(i)
        self.Edges = []
    def insert_edge(self,edge_start,edge_end,weight):
        if edge_start not in self.Vertices:
            print("Illegal Edge start point")
            return
        if edge_end not in self.Vertices:
            print("Illegal Edge end point")
            return
        self.Edges.append((edge_start,edge_end,weight))
    def print_AdjacencyList(self):
        AdjacencyList = {}
        for vertex in self.Vertices:
            AdjacencyList[vertex] = []
        for edge in self.Edges:
            AdjacencyList[edge[0]].append([edge[1],edge[2]])
        print(AdjacencyList)
    def print_AdjacencyMatrix(self):
        AdjacencyMatrix = []
        for i in range(len(self.Vertices)):
            AdjacencyMatrix.append([0]*len(self.Vertices))
        for edge in self.Edges:
            AdjacencyMatrix[edge[0]][edge[1]] = edge[2]
        print(AdjacencyMatrix)

class WeightedUnDirectedGraph:
    def __init__(self):
        self.graph = Graph(int(input("Enter Number of Nodes:")))
    def Menu(self):
        print("Enter <edge_start>,<edge_end>,<weight>  Enter -1,-1,-1 to exit")
        while True:
            a,b,c = list(map(int,input().split(",")))
            if a == -1 or b == -1 or c == -1:
                return
            self.graph.insert_edge(a,b,c)
            self.graph.insert_edge(b,a,c)



wudg = WeightedUnDirectedGraph()
wudg.Menu()
wudg.graph.print_AdjacencyList()
wudg.graph.print_AdjacencyMatrix()



        
