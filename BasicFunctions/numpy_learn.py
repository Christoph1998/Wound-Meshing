import numpy as np

test_array = np.array([[1, 8, 3], [3, 9, -3], [3, 4, 5], [3, 7, -5]], dtype=np.float64)

for row in test_array:
    print(row[0], row[1], row[2])  

# mask = np.where((test_array[:,0] > 1))[0]

# mask = np.where((test_array[:,0] > 1) & (test_array[:,1] > 5))[0]

# mask = np.where((test_array[:,0] > 1) & (test_array[:,1] > 5) & (test_array[:,2] < -2))[0]


mask_z = np.where(test_array[:,2] < -1)[0]
print(mask_z)
test_array_2 = test_array[mask_z]
print(test_array_2)

mask_y = np.where(test_array_2[:,1] > 8)[0]
print(mask_y)
test_array_3 = test_array_2[mask_y]
print(test_array_3)
# (test_array[:,1] < 3)
# (test_array[:,2] < 2)

