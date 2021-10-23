"""環境に関する関数，クラスなど
"""

import numpy as np
from math import pi, sin, cos, tan

from math_utils import rotate  



### 障害物を作る関数（高速） ###
def make_squre_obs(H, W, theta, position_center, dob = 0.2):
    """四角形の障害物を作成"""
    list_x = np.arange(-W/2, W/2, dob)
    top = np.array([list_x,
                    np.repeat([H/2], list_x.size)], np.float32)
    under = top - np.array([[0],
                            [H]], np.float32)
    list_y = np.arange(-H/2, H/2, dob)
    right = np.array([np.repeat([W/2], list_y.size),
                      list_y], np.float32)
    left = right - np.array([[W],
                             [0]], np.float32)
    position_o = np.concatenate([top, under, right, left], axis = 1)
    z = rotate(theta) @ position_o + position_center
    return z.T

def make_line_obs(W, theta ,position_center, dob = 0.2):
    """直線障害物を作成"""
    list_x = np.arange(-W/2, W/2, dob)
    line = np.array([list_x,
                     np.repeat([0], list_x.size)], np.float32)
    z = rotate(theta) @ line + position_center
    return z.T




# 障害物を動かす関数
def move_obstacle(t, obs_data):
    """移動障害物を動かす"""
    obs_box = []
    for i in np.arange(0, len(posio_o_move_data)-1, 2):
        center_posi = posi_o_move_data[i] + posi_o_move_data[i+1] * t
       
        obs_box.append()
    return posi_o_move



#### 障害物を作る関数 ###
#def make_squre_obs(H, W, theta, position_center, dob = 0.2):
#    """四角形の障害物を作成"""
#    position_o = np.array([[W/2,  W/2, -W/2, -W/2],
#                           [H/2, -H/2,  H/2, -H/2]], dtype = np.float32)
#    for i in np.arange(1, W/dob, 1):  # 上下辺
#        position_o = np.concatenate([position_o, np.array([[i*dob-W/2, H/2], 
#                                                           [i*dob-W/2, -H/2]], dtype = np.float32).T], axis=1)
#    for i in np.arange(1, H/dob, 1):  # 左右辺
#        position_o = np.concatenate([position_o, np.array([[W/2, i*dob-H/2], 
#                                                           [-W/2, i*dob-H/2]], dtype = np.float32).T], axis=1)
#    z = rotate(theta) @ position_o + position_center
#    return z.T

#def make_line_obs(W, theta ,position_center, dob = 0.2):
#    """直線の障害物を作成"""
#    position_o = np.array([[-W/2, W/2],
#                           [   0,   0]], dtype = np.float32)
#    for i in np.arange(1, W/dob, 1):
#        position_o = np.concatenate([position_o, np.array([[i*dob-W/2, 0], 
#                                                           [i*dob-W/2, 0]], dtype = np.float32).T], axis=1)
#    z = rotate(theta) @ position_o + position_center
#    return z.T



