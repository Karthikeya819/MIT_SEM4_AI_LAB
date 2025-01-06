# create two list X and Y with some set of numerical values. Compute Euclidean distance for corresponding
# values in X and Y. Store the distance values in a separate list and sort them using Bubble sort algorithm

import sys

X = [(0,0),(0,0),(0,0),(0,0)]
Y = [(0,4),(0,3),(0,2),(0,1)]
EDistances = []

if len(X) != len(Y):
    sys.exit(0)

# Caluculate Euclidian Distances
for i in range(len(X)):
    EDistances.append((abs(((X[i][0]-Y[i][0])**2) + ((X[i][1]-Y[i][1])**2)))**0.5)

# Bubble Sort
for j in range(len(EDistances)-1):
    for i in range(len(EDistances)-1-j):
        if EDistances[i+1] < EDistances[i]:
            EDistances[i+1],EDistances[i] = EDistances[i],EDistances[i+1]

print(EDistances)






