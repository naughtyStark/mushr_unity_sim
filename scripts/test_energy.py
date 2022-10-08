<<<<<<< HEAD
#!/usr/bin/env python3
=======
#!/usr/bin/env python
>>>>>>> f1df66dde220f9deec79a31c38407d9ce7d088f1
import numpy as np
from numpy.linalg import norm as norm
import math as m

N = 2 # number of cars
car_pos = np.zeros((N,3))
car_pos[0] = np.array([0, 5, -1.5708])
car_pos[1] = np.array([0, 0, 1.5708])

M = 2 # num points per waypoint
waypoint_set = np.zeros((N,M,3))
waypoint_set[0] = np.array([[1.5, 2.366, 0],[3, 1.5, -0.5]])
waypoint_set[1] = np.array([[1.5, 3.14, 0], [3, 4, 0.5]])

L1 = waypoint_set[0,1,:2] - waypoint_set[0,0,:2]
L2 = waypoint_set[0,0,:2] - car_pos[0,:2]
vec = 0.5*((L1/norm(L1)) + (L2/norm(L2)))
angle = m.atan2(vec[1], vec[0])
waypoint_set[0,0,2] = angle



def C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3):
    delX = t*t*KX1 + t*KX2 + KX3
    delY = t*t*KY1 + t*KY2 + KY3
    del2X = 2*t*KX1 + KX2
    del2Y = 2*t*KY1 + KY2
    denominator = delX*delX + delY*delY
    dummy = denominator
    denominator *= denominator*denominator
    denominator = np.sqrt(denominator)
    Curvature = ((delX*del2Y) - (delY*del2X))
    Curvature /= denominator
    return Curvature

def get_Curvature(X1,Y1,X2,Y2,X3,Y3,X4,Y4,t):
    Px = np.array([X1,X2,X3,X4])
    Py = np.array([Y1,Y2,Y3,Y4])
    KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
    KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
    KX2 = 6*X1 - 12*X2 + 6*X3
    KY2 = 6*Y1 - 12*Y2 + 6*Y3
    KX3 = 3*(X2 - X1)
    KY3 = 3*(Y2 - Y1)
    Curvature = C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3)
    return Curvature

def Bezier(P0, P1):
    d = norm(P0[:2] - P1[:2])
    t = np.arange(0,1,0.01)
    C = get_Curvature(P0[0],P0[1],
                    P0[0] + 0.35*d*m.cos(P0[2]), P0[1] + 0.35*d*m.sin(P0[2]),
                    P1[0] - 0.35*d*m.cos(P1[2]), P1[1] + 0.35*d*m.sin(P1[2]),
                    P1[0], P1[1],t)
    return np.max(C)
    
c1 = Bezier(car_pos[0], waypoint_set[0,0])
waypoint_set[0,1,2] = 0
c2 = Bezier(waypoint_set[0,0], waypoint_set[0,1])
print(c1, c2)