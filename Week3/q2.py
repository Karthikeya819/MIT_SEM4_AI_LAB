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
    def TopologicalOrder(self):
        InDegree = [0]*len(self.Vertices)
        AdjacencyMatrix = self.AdjacencyMatrix()
        for i in range(len(self.Vertices)):
            sum = 0
            for j in range(len(self.Vertices)):
                sum += AdjacencyMatrix[j][i]
            InDegree[i] = sum
        TopoOrder = sorted(enumerate(InDegree), key=lambda i: i[1])
        for elem in TopoOrder:
            print(elem[0],end=' ')
        print()

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

def DFS(graph,startindex=0):
    Stack = []
    AdajacencyList = graph.AdjacencyList()
    VisitedNodes = []
    Stack.append(graph.Vertices[startindex])
    while len(Stack) != 0:
        vertex = Stack.pop()
        if vertex not in VisitedNodes:
            for neighbour in AdajacencyList[vertex]:
                Stack.append(neighbour[0])
        VisitedNodes.append(vertex)
    return VisitedNodes

def NoOFCycles(gr,startindex):
    VisitedNodes = DFS(gr,startindex)
    print(VisitedNodes)
    sum = 0
    for vertex in set(VisitedNodes):
        sum += VisitedNodes.count(vertex)-1
    return sum

uwdg = UnWeightedDirectedGraph()
uwdg.Menu()
print(NoOFCycles(uwdg.graph,2))

# gr = Graph(6)
# gr.insert_edge(0,1,1)
# gr.insert_edge(1,2,1)
# gr.insert_edge(2,0,1)
# gr.insert_edge(0,2,1)
# gr.insert_edge(2,3,1)
# gr.insert_edge(3,3,1)
# print(NoOFCycles(gr,2))

    