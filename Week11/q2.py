from queue import Queue

def checkInvalidState(arr):
    for i in range(len(arr)):
        for j in range(i+1,len(arr)):
            if arr[i] == arr[j] or abs(arr[i] - arr[j]) == j-i:
                return True
    return False

queens_Size = 8
solutions = []
queue = Queue()
queue.put([])
while not queue.empty():
    solution = queue.get()
    if checkInvalidState(solution):
        continue
    if len(solution) == queens_Size:
        print(solution)
    for i in range(queens_Size):
        sol_copy = solution.copy()
        sol_copy.append(i)
        queue.put(sol_copy)