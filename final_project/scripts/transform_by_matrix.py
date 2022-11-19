#!/usr/bin/env python
import numpy as np

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix
    # Maybe add: If no angle it should be zeroes (atm it is nan)

p1w = [2, 2, 0]
p2w = [2, 4, 0]

p1h = [1, 1, 0]
p2h = [3, 1, 0]

vec1 = np.subtract(p2h, p1h)
vec2 = np.subtract(p2w, p1w)

print(vec1, vec2)

mat = rotation_matrix_from_vectors(vec1, vec2)
print(mat)
# Returns rotation of hidden frame from our world frame
# i.e. from world frame to hidden frame R_h^W (from example w = 0, h = 1)


# Calculate hidden frame:
angle_rad = -np.arcsin(mat[0,1])
angle_deg = angle_rad * 180 / np.pi
print("Angle:", angle_deg)

trans_mat = p1w - np.dot(mat, p1h)
print("Translation:", trans_mat)

# Example... Calculate point in world frame from hidden frame
p3h = [3, -3, 0]
p3w = np.dot(mat, p3h) + trans_mat
print(p3w)













# mat_2d = mat[:2, :2]
# print(mat_2d)

# p1h = p1h[:2]
# rot_mat = np.dot(mat_2d, p)
# print(rot_mat)


# p3w = [3, 6, 0]
# t = p3w[:2] - rot_mat

# print (t)

