"""停留点探索"""

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as anm
import matplotlib.patches as patches
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
import math
from math import pi, sin, cos, tan
import time
import datetime
#import scipy as sy
import pathlib

from math_utils import *  # いろんな関数のモジュール
from env_utils import *  # 障害物関連のモジュール
from compute_waypoint import *  # ウェイポイント計算モジュール
from RMP import *
from env import *


def minical():
    ### 定数 ###
    ## 車体形状
    L = 0.4  # 車体長さ[m]
    W = 0.25  # 車幅[m]
    x_local = np.array([[ L/2,    0],  # 鼻先
                        [ L/2, -W/2],  # 先頭右
                        [ L/4, -W/2],
                        [ 0,   -W/2],
                        [-L/4, -W/2],
                        [-L/2, -W/2],
                        [-L/2,    0],
                        [-L/2,  W/2],
                        [-L/4,  W/2],
                        [   0,  W/2],
                        [ L/4,  W/2],
                        [ L/2,  W/2]], dtype = np.float32)  # 制御点のローカル座標
    iend = x_local.shape[0]  # 制御点の数


    # 時間
    time_span = 200  # シミュレーション時間制限[s]
    time_interval = 1/10  # Δt = 25[fps]．固定（仮？）．


    ### シミュレーション環境 ###
    posi_o_fix, posi_o_move, waypoint, q_init = env6()

    posi_o = np.concatenate([posi_o_fix, posi_o_move], axis = 0)
    jend = posi_o.shape[0]
    print(jend)
    posi_g = waypoint[0:1, :]
    posi_all = np.concatenate([posi_o, posi_g], axis = 0)

    command_history = np.zeros((1, 5), np.float32)

    for x in np.arange(np.amin(posi_all[:, 0])-1, np.amax(posi_all[:, 0])+1, 1):
        for y in np.arange(np.amin(posi_all[:, 1])-1, np.amax(posi_all[:, 1])+1, 1):

            q = np.array([[x, y, 0]], np.float32).T
            dqdt = np.array([[0, 0, 0]], dtype = np.float32).T  # （車両中心の）速度．(2, 1)
            v_command = np.array([[0, 0]], dtype = np.float32).T # 制御指令
            dvdt_command = np.array([[0, 0]], dtype = np.float32).T # RMPの出力（制御指令速度）
            xi_global_all = xi_global_cal(x_local.T, q)  # 初期グローバル位置
            

            beta = beta_cal(v_command)
            J, T = transform_jacobi(L, q, v_command, beta)


            Last_Left_term = []
            Last_Right_term = []

            for i in np.arange(0, 12, 1):
                Jphii = Jphii_cal(L, W, q, x_local[i:i+1, :].T)
                dxidt_global = Jphii @ dqdt
                #fci, Aci = compute_cp_RMP_nonvec(xi_global_all[:, i:i+1], dxidt_global, posi_g, posi_o, jend, i)
                fci, Aci = compute_cp_RMP_vec(xi_global_all[:, i:i+1], dxidt_global, posi_g.T, posi_o.T, jend, i)
                Last_Left_term.append((Jphii @ J).T @ Aci @ (Jphii @ J))
                Last_Right_term.append((Jphii @ J).T @ Aci @ (fci - Jphii @ T))


            Last_Left_term = np.sum(Last_Left_term, axis = 0)
            Last_Right_term = np.sum(Last_Right_term, axis = 0)

            dvdt_command = np.linalg.pinv(Last_Left_term) @ Last_Right_term
        
            #データ保存
            command_history = np.concatenate([command_history, 
                                              np.concatenate([q.T, 
                                                              dvdt_command.T], axis = 1)], axis = 0)
            print(dvdt_command.T)
    print("計算終了")

    dvdt_mini = np.delete(command_history, np.where(command_history[:, 3:4] > 1)[1], axis = 0)
    #dvdt_mini = np.delete(dvdt_mini, np.where(dvdt_mini[:, 3:4] > -1e-1)[1], axis = 0)

    fig_mini = plt.figure(figsize=(10, 10))
    ax = fig_mini.add_subplot(111, projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("dvdt")
    ax.set_zlim(-1, 1)
    ax.scatter(posi_g[0, 0], posi_g[0, 1], 0, c = "red", s = 80)
    ax.scatter(posi_o[:, 0].T, posi_o[:, 1].T, 0, c = "black")
    ax.scatter(command_history[:, 0].T, command_history[:, 1].T, command_history[:, 3].T, s = 10, c = "b")
    ax.scatter(dvdt_mini[:, 0].T, dvdt_mini[:, 1].T, dvdt_mini[:, 3].T, s = 20, c = "m")
    #fig_mini.savefig("mini.png")
    plt.show()


if __name__ == "__main__":
    minical()

