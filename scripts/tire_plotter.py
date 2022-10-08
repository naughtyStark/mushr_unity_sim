#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

D = np.arange(0.5, 1.6, 0.1)
C = np.arange(1.0, 1.6, 0.1)
B = np.arange(1.5, 7.0, 0.1)

s = np.arange(0,2,0.01)

for i in range(len(D)):
    for j in range(len(C)):
        for k in range(len(B)):
            F = D[5] * np.sin( C[5] * np.arctan( B[k] * s))
            plt.plot(s, F, label="D= {}".format(D[i]))
# plt.legend()
plt.show()