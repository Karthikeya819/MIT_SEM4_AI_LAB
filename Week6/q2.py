# Write a Python program to Hill Climbing Search to solve the 8-Queens problem

def Queens_Score(queens):
    score = 0
    for p in range(len(queens)):
        for q in range(p+1,len(queens)):
            if queens[p] != queens[q] and abs(queens[p] - queens[q]) != q-p:
                score += 1
    return score

Current_State = [4,1,8,4,1,7,3,6]

while True:
    neighbours = []
    for i in range(len(Current_State)):
        if Current_State[i] < 8:
            next_state = Current_State.copy()
            next_state[i] += 1
            neighbours.append([Queens_Score(next_state),next_state])
        if Current_State[i] > 1:
            next_state = Current_State.copy()
            next_state[i] -= 1
            neighbours.append([Queens_Score(next_state),next_state])
    neighbours = sorted(neighbours, key=lambda x:x[0])
    if neighbours[-1][0] >= 28:
        print("Solution is ",neighbours[-1][1])
        break
    Current_State = neighbours[-1][1]
    

        

print(Queens_Score([5,1,8,4,2,7,3,6]))
