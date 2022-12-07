import numpy as np
import math

test_array = np.array([[1, 8, 3], [3, 9, -3], [3, 4, 5], [3, 7, -5]], dtype=np.float64)



x_min, x_max, x_dist = 0, 0, 0
y_min, y_max, y_dist = 0, 0, 0
z_min, z_max, z_dist = 0, 0, 0
   # calc min
mini = np.min(test_array)
maxi = np.max(test_array)

p1 = np.array(mini)
p2 = np.array(maxi)

squared_dist = np.sum((p1-p2)**2)
dist = np.sqrt(squared_dist)


print(dist)