#  Write a Python program to solve the water jug problem using Breadth First Search algorithm.

Jug_A_limit = 4
Jug_B_limit = 3
Goal_limit = 2

Queue = []
Queue.append([0,0,[]])
VisitedStates = []

def generateNextStates(node):
    global Jug_A_limit,Jug_B_limit,Queue
    x,y = node[0],node[1]
    if x < Jug_A_limit:
        arr_copy = node[2].copy()
        arr_copy.append("Fill Jug A with "+str(Jug_A_limit))
        Queue.append([Jug_A_limit,y,arr_copy])
    if y < Jug_B_limit:
        arr_copy = node[2].copy()
        arr_copy.append("Fill Jug B with "+str(Jug_B_limit))
        Queue.append([x,Jug_B_limit,arr_copy])
    if x > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Empty Jug A")
        Queue.append([0,y,arr_copy])
    if y > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Empty Jug B")
        Queue.append([x,0,arr_copy])
    if x+y >= Jug_A_limit and y > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Pour water from Jug B to Jug A until A is Full")
        Queue.append([Jug_A_limit,y-(Jug_A_limit-x),arr_copy])
    if x+y >= Jug_B_limit and x > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Pour water from Jug A to Jug B until B is Full")
        Queue.append([x-(Jug_B_limit-y),Jug_B_limit,arr_copy])
    if x +y <= Jug_A_limit and y > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Pour all water from Jug B to Jug A")
        Queue.append([x +y,0,arr_copy])
    if x+y <= Jug_B_limit and x > 0:
        arr_copy = node[2].copy()
        arr_copy.append("Pour all water from Jug A to Jug B")
        Queue.append([0,x +y,arr_copy])

while len(Queue) != 0:
    node = Queue.pop(-1)
    x,y = node[0],node[1]

    if [x,y] in VisitedStates:
        continue

    if x == Goal_limit or y ==  Goal_limit:
        for st in node[2]:
            print(st)
        exit(0)
    generateNextStates(node)
    VisitedStates.append([x,y])
print("No Solution Found!")