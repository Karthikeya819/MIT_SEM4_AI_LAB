# Implement the following graph using python. Print the adjacency list and adjacency matrix.
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

gr = Graph(4)
gr.insert_edge(0,1,1)
gr.insert_edge(0,2,1)
gr.insert_edge(1,2,3)
gr.insert_edge(2,3,4)
gr.insert_edge(3,0,5)
gr.print_AdjacencyList()
gr.print_AdjacencyMatrix()



        
