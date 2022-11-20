import numpy as np
import math 
s = math.sin
c = math.cos

t = math.pi/2

a = np.array([
    [c(t),-s(t),0,0],
    [s(t),c(t),0,0],
    [0,0,1,0],
    [0,0,0,1]
])
b = np.array([
    [1,0,0,0],
    [0,1,0,1],
    [0,0,1,0],
    [0,0,0,1]
])
c = np.array([
    [c(t),-s(t),0,0],
    [s(t),c(t),0,1],
    [0,0,1,0],
    [0,0,0,1]
])
d = a @ b 

p = np.array([0,1,0,1])

print(b @ a @ p )
print(d @ p)








