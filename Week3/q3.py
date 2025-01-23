Maze_StartPoint = 1
Maze_EndPoint = 4

Maze = {0: [1, 5], 1: [0, 2], 2: [1, 7], 3: [4], 4: [3, 9], 5: [0, 10], 6: [7], 7: [2, 6], 8: [9, 13], 
        9: [4, 8, 14], 10: [5, 11], 11: [10, 16], 12: [13], 13: [8, 12, 18], 14: [9, 19], 
        15: [16], 16: [11, 15, 17], 17: [16, 18], 18: [13, 17], 19: [14]}

def SearchMaze(Maze, Maze_StartPoint, Maze_EndPoint):
    VisitedNodes = []
    Stack = []
    Stack.append(Maze_StartPoint)
    while Stack:
        vertex = Stack.pop()
        if vertex not in VisitedNodes:
            VisitedNodes.append(vertex)
            if vertex == Maze_EndPoint:
                print(VisitedNodes)
                return
            for conn in Maze[vertex]:
                Stack.append(conn)

SearchMaze(Maze, Maze_StartPoint, Maze_EndPoint)
