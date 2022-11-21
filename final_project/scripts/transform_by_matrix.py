#!/usr/bin/env python
import numpy as np

p1w = [-0.3, -1.8, 0]
p2w = [-6.5, -2.6, 0]

p1h = [-3.08, 1.95, 0]
p2h = [2.67, 3.23, 0]


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

vec1 = np.subtract(p2h, p1h)
vec2 = np.subtract(p2w, p1w)

print(vec1, vec2)

mat = rotation_matrix_from_vectors(vec1, vec2)
print(mat)
# Returns rotation of hidden frame from our world frame
# i.e. from world frame to hidden frame R_h^W (from example w = 0, h = 1)


# Calculate hidden frame:
angle_rad = np.arctan2(mat[1,0],mat[0,0])

angle_deg = angle_rad * 180 / np.pi
print("Angle rad:", angle_rad)
print("Angle deg:", angle_deg)

trans_mat = p1w - np.dot(mat, p1h)
print(trans_mat, "Translation X:", trans_mat[0], "Translation Y:", trans_mat[1])



# Example... Calculate point in world frame from hidden frame
p3h = [3, -3, 0]
p3w = np.dot(mat, p3h) + trans_mat
print(p3w)








# def find_hidden_frame(self):
#         #Get values from 2 QR points and the according 2 world frame points
#         qrinfos = list(self.qr_info_initial.values())
#         hf0 = list(qrinfos[0].current_pos)+[0]
#         hf1 = list(qrinfos[1].current_pos)+[0]
#         vec_hf = np.subtract(hf1, hf0)

#         wfpose0 = qrinfos[0].world_pose.position
#         wfpose1 = qrinfos[1].world_pose.position
#         wf0 = [wfpose0.x, wfpose0.y, 0]
#         wf1 = [wfpose1.x, wfpose1.y, 0]
#         vec_wf = np.subtract(wf1, wf0)
        
#         def calculate_hidden_frame_transform(hidden_vector, world_vector):
#             a, b = (hidden_vector / np.linalg.norm(hidden_vector)).reshape(3), (world_vector / np.linalg.norm(world_vector)).reshape(3)
#             v = np.cross(a, b)
#             c = np.dot(a, b)
#             s = np.linalg.norm(v)
#             kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
#             rot_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
#             trans_matrix = wf1 - np.dot(rot_matrix, hf1)

#             trans_x = trans_matrix[0]
#             trans_y = trans_matrix[1]
#             angle_rad = -np.arcsin(rot_matrix[0,1])

#             return trans_x, trans_y, angle_rad

#         tf_x, tf_y, tf_yaw = calculate_hidden_frame_transform(vec_hf, vec_wf)

#         #Create a transform in the tf-tree
#         br = tf.TransformBroadcaster()
#         br.sendTransform((tf_x, tf_y, 0),
#                      tf.transformations.quaternion_from_euler(0 , 0, tf_yaw),
#                      rospy.Time.now(),
#                      "hidden_frame",
#                      "world")

#         pass
