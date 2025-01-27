graph = {
'A': {'B': 2, 'C': 3, 'D': 1},
'B': {'A': 2, 'C': 4, 'D': 2},
'C': {'A': 3, 'B': 4, 'D': 3},
'D': {'A': 1, 'B': 2, 'C': 3}
}
sales_man_start = "A"

Queue = []
Queue.append((sales_man_start,0))
while len(Queue) != 0:
    vertex,dist_trav = Queue.pop(0)
    if len(set(vertex)) >= len(graph) and vertex[-1] == sales_man_start:
        print(vertex,dist_trav)
        break
    for neighbour in graph[vertex[-1]]:
        Queue.append((vertex+neighbour,dist_trav+graph[vertex[-1]][neighbour]))