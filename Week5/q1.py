# Implement the following directed weighted graph using class, methods, and data structures of Python.
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
    def print_AdjacencyMatrix(self):
        AdjacencyMatrix = []
        for i in range(len(self.Vertices)):
            AdjacencyMatrix.append([0]*len(self.Vertices))
        for edge in self.Edges:
            AdjacencyMatrix[edge[0]][edge[1]] = edge[2]
        print(AdjacencyMatrix)


def BFS(graph,start,goal=[0]):
    VisitedNodes = []
    Queue = []
    AdajacencyList = graph.AdjacencyList()
    Queue.append([[start],0])
    while len(Queue) != 0:
        vertex,min_value,i_min = Queue[0][0],Queue[0][1],0
        for i in range(len(Queue)):
            if Queue[i][1] < min_value:
                min_value = Queue[i][1]
                vertex = Queue[i][0]
                i_min = i
        Queue.pop(i_min)
        if vertex[-1] in goal:
            VisitedNodes.append(vertex[-1])
            return vertex,min_value
        if vertex[-1] not in VisitedNodes:
            for neighb in AdajacencyList[vertex[-1]]:
                pah = vertex.copy()
                pah.append(neighb[0])
                Queue.append([pah,neighb[1]+min_value])
        VisitedNodes.append(vertex[-1])
    print("Goal Node Not Found!")

Nodes = input("Enter Nodes:").split(",")
mapping = {}
for i in range(len(Nodes)):
    mapping[Nodes[i]] = i
graph = Graph(len(Nodes))
print("Enter <edge_start>,<edge_end>,<weight>  Enter -1,-1,-1 to exit")
while True:
    a,b,c = input().split(",")
    if a == "-1" or b == "-1" or c == "-1":
        break
    graph.insert_edge(mapping[a],mapping[b],int(c))
start = mapping[input("Enter start:")]
goals = input("Enter Goals:").split(",")
for i in range(len(goals)):
    goals[i] = mapping[goals[i]]
path,cost = BFS(graph,start,goals)
for i in range(len(path)):
    path[i] = Nodes[path[i]]
print("The optimal path using BFS is ",path)
print("The cost of path is "+str(cost))

