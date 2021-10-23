"""シミュレーション環境
・いつかクラス化する
"""

import numpy as np
import math
from math import pi, sin, cos, tan


from math_utils import *  # いろんな関数のモジュール
from env_utils import *  # 障害物関連のモジュールfrom env_utils import *  # 障害物関連のモジュール


def env0():
    o1 = make_squre_obs(1, 1, np.pi/3, np.array([[2, 3]], dtype = np.float32).T)
    o2 = make_squre_obs(2, 3, 0, np.array([[5, 9]], dtype = np.float32).T)
    o3 = make_squre_obs(5, 1, 0, np.array([[7, 3]], dtype = np.float32).T)
    posi_o_fix = np.concatenate([o1, o2, o3], axis = 0)
    
    posi_o_move_vs = []
    posi_o_move_inits = []
    posi_o_move_init = []

    waypoint = np.array([[10, 10]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    return posi_o_fix, posi_o_move_vs, posi_o_move_inits, posi_o_move_init, waypoint, q_init


def env1():
    o1 = make_squre_obs(1, 1, pi/3, np.array([[2, 3]], dtype = np.float32).T)
    o2 = make_squre_obs(2, 3, 0, np.array([[5, 9]], dtype = np.float32).T)
    o3 = make_squre_obs(5, 1, pi/6, np.array([[6, 3]], dtype = np.float32).T)
    posi_o_fix = np.concatenate([o1, o2, o3], axis = 0)

    #om1_v = np.array([[0, 0.1]], np.float32).T
    #om1_init = np.array([[8.5, 0]], np.float32).T
    #om2_v = np.array([[0.05, 0]], np.float32).T
    #om2_init = np.array([[0, 6.4]], np.float32).T
    #posi_o_move_data = [om1_v, om1_init, om2_v, om2_init]

    #om1 = make_squre_obs(1, 0.5, pi/3, om1_init)
    #om2 = make_squre_obs(0.5, 0.5, 0, om2_init)
    #posi_o_move_init = np.concatenate([om1, om2], axis = 0)
    posi_o_move_vs = []
    posi_o_move_inits = []
    posi_o_move_init = []

    waypoint = np.array([[10, 10]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    return posi_o_fix, posi_o_move_vs, posi_o_move_inits, posi_o_move_init, waypoint, q_init


def env2():
    posi_o = np.array([[5, 5],
                       [5, 5.5],
                       [5, 6],
                       [5, 6.5],
                       [8, 4],
                       [9, 9],
                       [1, 1],
                       [1, 2],
                       [2, 3]], dtype = np.float32)  # (2, jend)
    posi_g = np.array([[10, 10]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    return


def env3():
    # 環境3（障害物一個）
    posi_o = np.array([[1, 1.3],
                       [1,   1],
                       [1, 0.5],
                       [1, 0.6],
                       [1, 0.8]], dtype = np.float32)
    posi_g = np.array([[5, 5]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    return posi_o_fix, posi_o_move, waypoint, q_init


def env4():
    # 環境4
    posi_o = np.array([[5, 5]], dtype = np.float32)
    posi_g = np.array([[10, 10]], dtype = np.float32)
    q_init = np.array([[0, 0, pi/2]], dtype = np.float32).T
    return posi_o_fix, posi_o_move, waypoint, q_init


def env5():
    # 環境5（曲がった廊下）
    o1 = make_line_obs(5, 0, np.array([[2.5, 4]], dtype = np.float32).T)
    o2 = make_line_obs(3.4, 0, np.array([[3.5/2, 3]], dtype = np.float32).T)
    o3 = make_line_obs(0.5, 0, np.array([[4.75, 3]], dtype = np.float32).T)
    o4 = make_line_obs(1, pi/2, np.array([[5, 3.5]], dtype = np.float32).T)
    o5 = make_line_obs(3, pi/2, np.array([[3.5, 1.5]], dtype = np.float32).T)
    o6 = make_line_obs(3, pi/2, np.array([[4.5, 1.5]], dtype = np.float32).T)
    posi_o_fix = np.concatenate([o1, o2, o3, o4, o5, o6], axis = 0)
    #posi_o_move = make_squre_obs(H = 1.5, W = 1.5, theta = 0, position_center = np.array([[10, 1]]).T, dob = 0.2)

    posi_o_move_vs = []
    posi_o_move_inits = []
    posi_o_move_init = []

    q_init = np.array([[2.5, 3.5, 0]], dtype = np.float32).T
    waypoint = np.array([[4.1, 2]], dtype = np.float32)
    return posi_o_fix, posi_o_move_vs, posi_o_move_inits, posi_o_move_init, waypoint, q_init


def env6():
    # 環境６（廊下，動的障害物）
    #o1 = make_line_obs(12, 0, np.array([[5, 2]], dtype = np.float32).T, dob = 1)
    #o2 = make_line_obs(12, 0, np.array([[5, -2]], dtype = np.float32).T, dob = 1)
    ##o3 = np.array([[50, 50]], dtype = np.float32)
    #posi_o_fix = np.concatenate([o1, o2], axis = 0)
    posi_o_fix = np.array([[0, 3]], dtype = np.float32)
    mo1 = make_squre_obs(H = 1.5, W = 1.5, theta = 0, position_center = np.array([[5, 0]]).T, dob = 0.2)
    posi_o_move = mo1
    #posi_o_move = np.array([[0, 3]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    waypoint = np.array([[8, 1]], dtype = np.float32)
    return posi_o_fix, posi_o_move, waypoint, q_init


def env7():
    # 環境7（動的障害物，壁無し）
    posi_o_fix = []
    om1_v = np.array([[-0.05, 0]], np.float32).T
    om1_init = np.array([[7, 0]], np.float32).T
    posi_o_move_vs = [om1_v]
    posi_o_move_inits = [om1_init]
    mo1 = make_squre_obs(H = 1.5, W = 1.5, theta = 0, position_center = om1_init, dob = 0.2)
    posi_o_move_init = mo1
    waypoint = np.array([[10, 0.5]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    return posi_o_fix, posi_o_move_vs, posi_o_move_inits, posi_o_move_init, waypoint, q_init


def env8():
    # 環境8（障害物なし，ウェイポイント試験用）．障害物回避RMPを切る必要あり．
    posi_o_fix = []
    posi_o_move = np.array([[0, 5]], dtype = np.float32)
    q_init = np.array([[0, 0, -pi/2]], dtype = np.float32).T
    waypoint = np.array([[2, 2],
                         [4, -2],
                         [6, 2],
                         [8, -2],
                         [10, 2]], dtype = np.float32)
    return posi_o_fix, posi_o_move, waypoint, q_init


def env9():
    """環境9（論文みたいなやつ）"""
    posi_o_fix = make_squre_obs(H = 2, W = 2.8, theta = -pi/4, position_center = np.array([[4, 0]]).T, dob = 0.2)
    posi_o_move = np.array([[0, 3]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    waypoint = np.array([[10, -2]], dtype = np.float32)
    return posi_o_fix, posi_o_move, waypoint, q_init


def env10():
    """ウェイポイントチェック2"""
    posi_o_fix = np.array([[1, 0.1]], dtype = np.float32)
    posi_o_move = np.array([[0, 5]], dtype = np.float32)
    q_init = np.array([[0, 0, 0]], dtype = np.float32).T
    waypoint = np.array([[3, 0]], dtype = np.float32)
    return posi_o_fix, posi_o_move, waypoint, q_init


def env11():
    """要旨用
    ・いろんな障害物を動かす
    """

