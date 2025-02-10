# Write python code to find maximum value of f(x) where -10 <= x <= 10) using Hill Climbing method.

import random

def F(x):
    return 4*x + 10 - (x**2)

Range_min = -10
Range_max = 10
step_value = 0.1

Range = list(range(Range_min,Range_max,1))
Current_Value = Range[int(random.random()*len(Range))]

while True:
    if Current_Value < Range_min or Current_Value > Range_max:
        print("Search value reached out of range")
        break
    neighb_1,neighb_2 = Current_Value - step_value,Current_Value + step_value
    f_Current_Value,f_neighb_1,f_neighb_2 = F(Current_Value),F(neighb_1),F(neighb_2)
    print(neighb_1,neighb_2)
    if f_neighb_2 >= f_Current_Value and f_neighb_1 <= f_Current_Value:
        Current_Value = neighb_2
    elif f_neighb_1 >= f_Current_Value and f_neighb_2 <= f_Current_Value:
        Current_Value = neighb_1
    elif f_neighb_1 <= f_Current_Value and f_neighb_2 <= f_Current_Value:
        print("Solution is " + str(Current_Value))
        break