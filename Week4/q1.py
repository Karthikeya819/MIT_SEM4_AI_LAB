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
    def AdjacencyList(self):
        AdjacencyList = {}
        for vertex in self.Vertices:
            AdjacencyList[vertex] = []
        for edge in self.Edges:
            AdjacencyList[edge[0]].append([edge[1],edge[2]])
        return AdjacencyList
    def AdjacencyMatrix(self):
        AdjacencyMatrix = []
        for i in range(len(self.Vertices)):
            AdjacencyMatrix.append([0]*len(self.Vertices))
        for edge in self.Edges:
            AdjacencyMatrix[edge[0]][edge[1]] = edge[2]
        return AdjacencyMatrix

class UnWeightedDirectedGraph:
    def __init__(self):
        self.graph = Graph(int(input("Enter Number of Nodes:")))
    def Menu(self):
        print("Enter <edge_start>,<edge_end>  Enter -1,-1 to exit")
        while True:
            a,b = list(map(int,input().split(",")))
            if a == -1 or b == -1:
                return
            self.graph.insert_edge(a,b,1)

def BFS(graph):
    VisitedNodes = []
    Queue = []
    AdajacencyList = graph.AdjacencyList()
    for newvertex in graph.Vertices:
        Queue.append(newvertex)
        while len(Queue) != 0:
            vertex = Queue.pop(0)
            if vertex not in VisitedNodes:
                for neighbour in AdajacencyList[vertex]:
                    Queue.append(neighbour[0])
            VisitedNodes.append(vertex)
    return VisitedNodes

def TopologicalOrder(uwdg):
        graph = uwdg.graph
        VisitedNodes = BFS(graph)
        TopoOrder = []
        for vertex in graph.Vertices:
            TopoOrder.append(VisitedNodes.count(vertex)-1)
        TopoOrder = sorted(enumerate(TopoOrder), key=lambda i: i[1])
        for elem in TopoOrder:
            print(elem[0],end=' ')
        print()
uwdg = UnWeightedDirectedGraph()
uwdg.Menu()
TopologicalOrder(uwdg)


