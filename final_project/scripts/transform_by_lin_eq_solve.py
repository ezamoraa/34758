import numpy as np

pw1 = [2,2,]
pw2 = [2,4]

ph1 = [1,1]
ph2 = [3,1]

def calculate_hidden_frame(p_world, p_hidden):
    a = np.array([
        [p_hidden[0,0], -p_hidden[0,1], 1, 0],
        [p_hidden[0,1], p_hidden[0,0], 0, 1],
        [p_hidden[1,0], -p_hidden[1,1], 1, 0],
        [p_hidden[1,1], p_hidden[1,0], 0, 1]]
        )
    b = np.array([
        p_world[0,0],
        p_world[0,1],
        p_world[1,0],
        p_world[1,1]]
        )
    x = np.linalg.solve(a, b)

    translate_x = x[2]
    translate_y = x[3]
    
    theta = np.arctan2(x[1], x[0])

    return translate_x, translate_y, theta



def calculate_point_from_hidden_frame(p_hidden, translate_x, translate_y, theta):
    new_x = np.cos(theta) * p_hidden[0] - np.sin(theta) * p_hidden[1] + translate_x
    new_y = np.sin(theta) * p_hidden[0] + np.cos(theta) * p_hidden[1] + translate_y

    new_point_wf = [new_x, new_y]

    return new_point_wf

p_world = np.array([pw1, pw2])
p_hidden = np.array([ph1, ph2])

translate_x, translate_y, theta = calculate_hidden_frame(p_world, p_hidden)

print(theta / np.pi * 180)
print(translate_x)
print(translate_y)





# Example... Calculate point in world frame from hidden frame
p_new_h = [3, -3]
new = calculate_point_from_hidden_frame(p_new_h, translate_x, translate_y, theta)
print(new)








