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

def AStar(graph,start,goals,Heuristic):
    VisitedNode,Queue,AdjacencyList = [],[],graph.AdjacencyList()
    Queue.append([[start],Heuristic[start],0])
    while len(Queue) != 0:
        vertex,min_value,min_path,i_min = Queue[0][0],Queue[0][1],Queue[0][2],0
        for i in range(len(Queue)):
            if Queue[i][1] < min_value:
                min_value = Queue[i][1]
                vertex = Queue[i][0]
                min_path = Queue[0][2]
                i_min = i
        Queue.pop(i_min)
        if vertex[-1] in goals:
            VisitedNode.append(vertex[-1])
            return vertex,min_path
        if vertex not in VisitedNode:
            for neigh in AdjacencyList[vertex[-1]]:
                pah = vertex.copy()
                pah.append(neigh[0])
                Queue.append([pah,Heuristic[neigh[0]]+min_path+neigh[1],neigh[1]+min_path])


Nodes = input("Enter Nodes:").split(",")
Heuristic = list(map(int,input("Enter Heuristic Values: ").split(",")))
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
path,cost = AStar(graph,start,goals,Heuristic)
for i in range(len(path)):
    path[i] = Nodes[path[i]]
print("The optimal path using BFS is ",path)
print("The cost of path is "+str(cost))