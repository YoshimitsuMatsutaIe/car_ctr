"""ぶん回す用のメイン
・ほぼmain_funcと同じ
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anm
import matplotlib.patches as patches
import matplotlib.cm as cm
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
from car_dynamics import *
from car_model import *


def main_exp_simu():
    """メインシミュレーション
    ・回すときはtimeとかanimeとか切る
    """

    Time_start = time.time()  # 実行時間計測


    ### 定数 ###
    ## 車体形状
    #lf = 0.9560  # 正面から重心までの距離[m]
    #lr = 1.434
    #L = lf + lr  # 車体長さ[m]
    #W = 0.25 * L / 0.4  # 車幅[m]
    L = 0.4
    W = 0.25

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

    #x_local = (x_local.T + np.array([[L/2, 0]], dtype = np.float32).T).T  # ドリフト無しモデルのときはこれ追加

    iend = x_local.shape[0]  # 制御点の数


    # 時間
    time_span = 100  # シミュレーション時間制限[s]
    time_interval = 1/10  # Δt = 25[fps]．固定（仮？）．


    ### シミュレーション環境 ###
    posi_o_fix, posi_o_move, waypoint, q_init = env6()


    ### 初期値 ###
    q = q_init  # （車両中心の）初期位置．(3, 1)
    dqdt = np.array([[0, 0, 0]], dtype = np.float32).T  # （車両中心の）速度．(2, 1)
    v_command = np.array([[0, 0]], dtype = np.float32).T # 制御指令
    dvdt_command = np.array([[0, 0]], dtype = np.float32).T # RMPの出力（制御指令速度）
    xi_global_all = xi_global_cal(x_local.T, q)  # 初期グローバル位置
    posi_g = waypoint[0:1, :]
    waypoint_count = 0  #  現在のウェイポイント番号

    beta_slip = np.array([[0]], np.float32)
    d2thetadt2 = np.array([[0]], np.float32)

    ### データ格納 ###
    state_history = np.concatenate([np.array([[0]], dtype = np.float32), 
                                    q.T, 
                                    dqdt.T, 
                                    v_command.T, 
                                    np.array([[beta_cal(v_command)]], dtype = np.float32), 
                                    dvdt_command.T, 
                                    xi_global_all.T.reshape([1, 24]), ], axis = 1)
    RMPc_history = np.zeros((1, 74), dtype = np.float32)  # fとAを格納
    obs_history = posi_o_move.reshape(1, posi_o_move.shape[0] * 2)

    #model_history = np.zeros((1, 3), dtype = np.float32)  # すべり角と各加速度を格納


    print("mainループ開始までの時間", time.time() - Time_start)


    ### シミュレーション本体 ###
    print("シミュレーション実行中...")
    Time_sim_start = time.time()

    for t in np.arange(time_interval, time_span + time_interval, time_interval, dtype = np.float32):

        time_freeze = 10  # 停留判定時間．この時間の動かなかったらbreak
        print(t)

        ## 障害物を動かす
        mo1_center = np.array([[10 - 0.5 * 10, 0]], dtype = np.float32).T
        mo1 = make_squre_obs(H = 1.5, W = 1.5, theta = 0, position_center = mo1_center, dob = 0.2)
        posi_o_move = mo1

        # 環境8のとき
        #posi_o_move = np.array([[0, 5]], dtype = np.float32)
    
        posi_o = np.concatenate([posi_o_fix, posi_o_move], axis = 0)
        jend = posi_o.shape[0]

        stationary_judg_index = max(int((t - time_freeze) / time_interval), 0)  # 停留判定用インデックス


        if waypoint_count > waypoint.shape[0]-1:
            error_r = 0.01
        else:
            error_r = 0.3

        if np.linalg.norm(posi_g.T - xi_global_all[:, 0:1]) < error_r:  # ウェイポイント到達判定
            print("ウェイポイント", posi_g, "に到達")
            waypoint_count += 1  # カウントを1増やす
            if waypoint_count > waypoint.shape[0]-1:
                print("目標到達！")
                break
            else:
                posi_g = waypoint[waypoint_count:waypoint_count+1, :]  # 次のウェイポイントを目標に設定

        elif v_command[0, 0] > 10:  # 発散判定
            print("制御指令が発散")
            break

        elif (np.linalg.norm(q[0:2, :] - state_history[stationary_judg_index:stationary_judg_index + 1, 1:3].T) < 0.001) and (t > time_freeze):  # 停留判定
            print("停留点")
            break

        elif np.min(np.linalg.norm(q[0:2, :] - posi_o.T, axis = 0)) < L / 2:  # 衝突判定
            print("障害物に衝突")
            break

        else:
            # RMPで制御指令vを計算

            beta = beta_cal(v_command)
            J, T = transform_jacobi(L, q, v_command, beta)

            fc_history = np.array([[t]], dtype = np.float32)
            Ac_history = np.array([[t]], dtype = np.float32)

            Last_Left_term = []
            Last_Right_term = []

            for i in np.arange(0, 12, 1):
                Jphii = Jphii_cal(L, W, q, x_local[i:i+1, :].T)
                dxidt_global = Jphii @ dqdt
                fci, Aci = compute_cp_RMP_nonvec(xi_global_all[:, i:i+1], dxidt_global, posi_g, posi_o, jend, i)
                #fci, Aci = compute_cp_RMP_vec(xi_global_all[:, i:i+1], dxidt_global, posi_g.T, posi_o.T, jend, i)
                fc_history = np.concatenate([fc_history, fci.T], axis = 1)
                Ac_history = np.concatenate([Ac_history, Aci.reshape([1, 4])], axis = 1)
                Last_Left_term.append((Jphii @ J).T @ Aci @ (Jphii @ J))
                Last_Right_term.append((Jphii @ J).T @ Aci @ (fci - Jphii @ T))
                #Last_Right_term.append((Jphii @ J).T @ Aci @ (fci - Jphii @ T - dJphiidt_cal(q, dqdt, x_local[i:i+1, :].T) @ dqdt))  # 曲率項を無視しないパターン．変わってないような


            Last_Left_term = np.sum(Last_Left_term, axis = 0)
            Last_Right_term = np.sum(Last_Right_term, axis = 0)

            dvdt_command = np.linalg.pinv(Last_Left_term) @ Last_Right_term
        

            ## 以下固定
            v_command = v_command + dvdt_command * time_interval  # 制御指令
            beta = beta_cal(v_command)

            # 制御指令vで状態遷移（ルンゲクッタで車両状態方程式を数値積分して）
            dqdt = transform_velocity(L, q, v_command, beta)  #車両中心速度を計算
            runge_kutta_sol = carsimu(q, dqdt, v_command, beta_slip, d2thetadt2)
            #print(runge_kutta_sol)
            q = runge_kutta_sol[:, 0:3].T
            beta_slip = runge_kutta_sol[:, 3:4]
            d2thetadt2 = runge_kutta_sol[:, 4:5]
            xi_global_all = xi_global_cal(x_local.T, q)
        
            #データ保存
            state_history = np.concatenate([state_history, 
                                            np.concatenate([np.array([[t]], dtype = np.float32), 
                                                            q.T, 
                                                            dqdt.T, 
                                                            v_command.T, 
                                                            np.array([[beta]], dtype = np.float32), 
                                                            dvdt_command.T, 
                                                            xi_global_all.T.reshape([1, 24])], axis = 1)], axis = 0)
            RMPc_history = np.concatenate([RMPc_history, 
                                           np.concatenate([fc_history, 
                                                           Ac_history], axis = 1)], axis = 0)

            obs_history = np.concatenate([obs_history, 
                                          posi_o_move.reshape(1, posi_o_move.size)], axis = 0)

    tend = t  # シミュレーション終了時刻


    print("シミュレーション終了")
    print("シミュレーション実行時間", time.time() - Time_sim_start)

    Time_savedata_start = time.time()


    ### データ保存 ###
    date_now = datetime.datetime.now()  # 日付取得
    header = date_now.strftime('%Y-%m-%d--%H-%M-%S')
    state_history_file_name = header + '__State_history_temp.csv'
    RMPc_history_file_name = header + '__RMP_history_temp.csv'
    #save_path = pathlib.Path(r'D:\control2020\RMP_exp\Exp_data_home\2021_01_23')  # 家
    save_path = pathlib.Path(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\2021_01_25')  # 大学
    path1 = save_path / state_history_file_name
    path2 = save_path / RMPc_history_file_name
    print("csvファイルでデータ保存中...")
    np.savetxt(path1, state_history, delimiter = ",")
    np.savetxt(path2, RMPc_history, delimiter = ",")

    print("データ保存実行時間", time.time() - Time_savedata_start)

    Time_fig_start = time.time()



    ### グラフ化 ###
    # グラフサイズ決定
    posi_all = np.concatenate([waypoint, posi_o, state_history[:, 1:3]], axis = 0)
    fig_xmin = np.amin(posi_all[:, 0:1])
    fig_xmax = np.amax(posi_all[:, 0:1])
    fig_ymin = np.amin(posi_all[:, 1:2])
    fig_ymax = np.amax(posi_all[:, 1:2])

    fig_max_length = 13

    x_scale = fig_xmax - fig_xmin
    y_scale = fig_ymax - fig_ymin

    if x_scale >= y_scale:
        fig_W = fig_max_length
        fig_L = fig_max_length * y_scale / x_scale
    else:
        fig_W = fig_max_length * x_scale / y_scale
        fig_L = fig_max_length



    ##軌跡（グラデーション付き）
    fig_Tra, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.scatter(waypoint[waypoint.shape[0]-1, 0], waypoint[waypoint.shape[0]-1, 1], 
               s = 200, label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')
    ax.scatter(waypoint[0:waypoint.shape[0]-1, 0].T, waypoint[0:waypoint.shape[0]-1, 1].T, 
               s = 150, label = 'way point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'orange')
    ax.scatter(posi_o[:, 0].T, posi_o[:, 1].T, 
               s = 80, label = 'obstacle points', marker = '+', c = 'black', alpha = 1)
    v_max = np.amax(state_history[:, 7])
    v_min = np.amin(state_history[:, 7])
    for i in np.arange(0, state_history.shape[0], 1):
        ax.plot(state_history[i:i+2, 1].T, 
                state_history[i:i+2, 2].T, 
                color = cm.winter((state_history[i, 7] - v_min) / (v_max - v_min)), 
                markersize = 1.5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)
    ax.set_xlim(fig_xmin - 1, fig_xmax + 1)
    ax.set_ylim(fig_ymin - 1, fig_ymax + 1)
    ax.legend(loc='best')
    ax.set_aspect('equal')

    fig_Tra_name = header + "__Trajectory_gra.png"
    fig_Tra.savefig(save_path / fig_Tra_name)


    ##速度など
    fig_State,axes = plt.subplots(nrows=3,ncols=4,figsize=(15,10))
    axes[0,0].plot(state_history[:, 0].T, state_history[:, 1].T, marker="")
    axes[0,0].set_title('position_x')
    axes[0,0].set_xlabel('t')
    axes[0,0].set_ylabel('x')
    axes[0,0].set_xlim(0, tend)
    #axes[0,0].set_ylim(-1, 11)
    axes[0,0].grid(True)

    axes[1,0].plot(state_history[:, 0].T, state_history[:, 2].T, marker="")
    axes[1,0].set_title('position_y')
    axes[1,0].set_xlabel('t')
    axes[1,0].set_ylabel('y')
    axes[1,0].set_xlim(0, tend)
    #axes[1,0].set_ylim(-1, 11)
    axes[1,0].grid(True)

    #axes[2,0].plot(state_history[:, 0].T, np.mod(state_history[:, 3].T, 2*pi), marker="")
    axes[2,0].plot(state_history[:, 0].T, state_history[:, 3].T, marker="")
    axes[2,0].set_title('position_theta')
    axes[2,0].set_xlabel('t')
    axes[2,0].set_ylabel('theta')
    axes[2,0].set_xlim(0, tend)
    #axes[2,0].set_ylim(0, 2*pi)
    axes[2,0].set_ylim(-pi, pi)
    axes[2,0].grid(True)

    axes[0,1].plot(state_history[:, 0].T, state_history[:, 4].T, marker="")
    axes[0,1].set_title('velocity_dxdt')
    axes[0,1].set_xlabel('t')
    axes[0,1].set_ylabel('dxdt')
    axes[0,1].set_xlim(0, tend)
    #axes[0,1].set_ylim(-alpha_g[0] / beta_g[0], alpha_g[0] / beta_g[0])
    axes[0,1].grid(True)

    axes[1,1].plot(state_history[:, 0].T, state_history[:, 5].T, marker="")
    axes[1,1].set_title('velocity_dydt')
    axes[1,1].set_xlabel('t')
    axes[1,1].set_ylabel('dydt')
    axes[1,1].set_xlim(0, tend)
    #axes[1,1].set_ylim(-alpha_g[0] / beta_g[0], alpha_g[0] / beta_g[0])
    axes[1,1].grid(True)

    #axes[2,1].plot(state_history[:, 0].T, np.mod(state_history[:, 6].T, 2*pi), marker="")
    axes[2,1].plot(state_history[:, 0].T, state_history[:, 6].T, marker="")
    axes[2,1].set_title('velocity_theta')
    axes[2,1].set_xlabel('t')
    axes[2,1].set_ylabel('dthetadt')
    axes[2,1].set_xlim(0, tend)
    #axes[2,1].set_ylim(0, 2*pi)
    #axes[2,1].set_ylim(-pi, pi)
    axes[2,1].grid(True)

    axes[0,2].plot(state_history[:, 0].T, state_history[:, 7].T, marker="")
    axes[0,2].set_title('forword velocity_v')
    axes[0,2].set_xlabel('t')
    axes[0,2].set_ylabel('v')
    axes[0,2].set_xlim(0, tend)
    #axes[0,2].set_ylim(-alpha_g[0] / beta_g[0], alpha_g[0] / beta_g[0])
    axes[0,2].grid(True)

    #axes[1,2].plot(state_history[:, 0].T, np.mod(state_history[:, 8].T, 2*pi), marker="")
    axes[1,2].plot(state_history[:, 0].T, state_history[:, 8].T, marker="")
    axes[1,2].set_title('steering angle_ξ')
    axes[1,2].set_xlabel('t')
    axes[1,2].set_ylabel('xi')
    axes[1,2].set_xlim(0, tend)
    #axes[1,2].set_ylim(0, 2*pi)
    axes[2,1].set_ylim(-pi, pi)
    axes[1,2].grid(True)

    #axes[2,2].plot(state_history[:, 0].T, np.mod(state_history[:, 9].T, 2*pi), marker="")
    axes[2,2].plot(state_history[:, 0].T, state_history[:, 9].T, marker="")
    axes[2,2].set_title('beta')
    axes[2,2].set_xlabel('t')
    axes[2,2].set_ylabel('beta')
    axes[2,2].set_xlim(0, tend)
    #axes[2,2].set_ylim(0, 2*pi)
    axes[2,2].grid(True)

    axes[0,3].plot(state_history[:, 0].T, state_history[:, 10].T, marker="")
    axes[0,3].set_title('forword accerlation_dvdt')
    axes[0,3].set_xlabel('t')
    axes[0,3].set_ylabel('dvdt')
    axes[0,3].set_xlim(0, tend)
    #axes[0,3].set_ylim(-alpha_g[0], alpha_g[0])
    axes[0,3].grid(True)

    #axes[1,3].plot(state_history[:, 0].T, np.mod(state_history[:, 11].T, 2*pi), marker="")
    axes[1,3].plot(state_history[:, 0].T, state_history[:, 11].T, marker="")
    axes[1,3].set_title('steering angle velocity_dξdt')
    axes[1,3].set_xlabel('t')
    axes[1,3].set_ylabel('dxidt')
    axes[1,3].set_xlim(0, tend)
    #axes[1,3].set_ylim(0, 2*pi)
    #axes[2,1].set_ylim(-pi, pi)
    axes[1,3].grid(True)

    axes[2,3].axis('off')

    fig_State_name = header + "__State.png"
    fig_State.savefig(save_path / fig_State_name)


    print("グラフ化時間", time.time() - Time_fig_start)

    Time_metric_start = time.time()

    # アニメ
    print("計量の等高線を計算中．．．")
    met = np.zeros((1, 37), dtype = np.float32)
    for l in np.arange(1, state_history.shape[0], 1):
        met_before = np.array([[l * time_interval]], dtype = np.float32)
        for i in np.arange(0, 12, 1):
            metric = np.array([[RMPc_history[l, 26 + 4 * i],     RMPc_history[l, 26 + 4 * i + 1]],
                               [RMPc_history[l, 26 + 4 * i + 2], RMPc_history[l, 26 + 4 * i + 3]]], dtype = np.float32)
            eigvals, eigvecs = np.linalg.eig(metric)  # 計量の固有値と固有ベクトルを計算
            if np.any(np.iscomplex(eigvals)) or np.any(eigvals <= 1e-3): # 正定対称じゃない．リーマン計量じゃないのでスキップ
                metric_axes_lengths = np.array([0, 0], dtype = np.float32)
                metric_angle = 0
            else:  # リーマン計量だから描写
                axes_lengths = 1.0 / np.sqrt(eigvals) * 0.1
                max_len = max(axes_lengths)
                scale = min(2.0 / max_len, 1.0)
                metric_axes_lengths = axes_lengths * scale
                metric_angle = np.rad2deg(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]) + state_history[l, 3])  # 楕円の傾き

            SSS = np.array([[metric_axes_lengths[0], metric_axes_lengths[1], metric_angle]], dtype = np.float32)
            met_before = np.concatenate([met_before, SSS], axis = 1)
        met = np.concatenate([met, met_before], axis = 0)

    print("加速度ベクトルを計算中．．．")
    fvec = np.zeros((2, 25), dtype = np.float32)
    for l in np.arange(1, state_history.shape[0], 1):
        dim = 5  # ベクトル長さ
        fvec_before = l * time_interval * np.array([[1, 1]], dtype = np.float32).T
        for i in np.arange(0 ,12 ,1):
            accel_vec  = np.array([[state_history[l, 12 + 2 * i],                                    state_history[l, 13 + 2 * i]],
                                   [state_history[l, 12 + 2 * i] + RMPc_history[l, 1 + 2 * i] * dim, state_history[l, 13 + 2 * i] + RMPc_history[l, 2 + 2 * i] * dim]], dtype = np.float32)
            #print(accel_vec)
            fvec_before = np.concatenate([fvec_before, accel_vec], axis = 1)
        fvec = np.concatenate([fvec, fvec_before], axis = 0)
  

    print("計量，加速度計算時間", time.time() - Time_metric_start)

    Time_ani_start = time.time()

    print("1-アニメ化中．．．")

    fig_ani = plt.figure(figsize = (fig_W, fig_L))
    ax = fig_ani.add_subplot(111, xlim=(fig_xmin - 1, fig_xmax + 1), ylim=(fig_ymin - 1, fig_ymax + 1))
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    #ax.legend()

    ax.scatter(waypoint[waypoint.shape[0]-1, 0], waypoint[waypoint.shape[0]-1, 1], 
               s = 200, label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')
    ax.scatter(waypoint[0:waypoint.shape[0]-1, 0].T, waypoint[0:waypoint.shape[0]-1, 1].T, 
               s = 150, label = 'way point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'orange')
    ax.scatter(posi_o_fix[:, 0].T, posi_o_fix[:, 1].T, 
               s = 80, label = 'obstacle points', marker = '+', c = 'black', alpha = 1)  # 障害物点


    center, = ax.plot([], [], '-b', lw=1)  # 中心軌跡

    # 以下制御点
    cpoint0, = ax.plot([], [], '.-.r', lw=1)  # 鼻先
    cpoint1, = ax.plot([], [], '.-.g', lw=1)
    cpoint2, = ax.plot([], [], '.-.g', lw=1)
    cpoint3, = ax.plot([], [], '.-.g', lw=1)
    cpoint4, = ax.plot([], [], '.-.g', lw=1)
    cpoint5, = ax.plot([], [], '.-.g', lw=1)
    cpoint6, = ax.plot([], [], '.-.k', lw=1)
    cpoint7, = ax.plot([], [], '.-.m', lw=1)
    cpoint8, = ax.plot([], [], '.-.m', lw=1)
    cpoint9, = ax.plot([], [], '.-.m', lw=1)
    cpoint10, = ax.plot([], [], '.-.m', lw=1)
    cpoint11, = ax.plot([], [], '.-.m', lw=1)

    cpoints = [cpoint0, cpoint1, cpoint2, cpoint3, cpoint4, cpoint5, cpoint6, cpoint7, cpoint8, cpoint9, cpoint10, cpoint11]  # リスト化

    # 制御点ポリシー
    f0, = ax.plot([], [], '-r', lw = 2)  # 鼻先
    f1, = ax.plot([], [], '-g', lw = 2)
    f2, = ax.plot([], [], '-g', lw = 0.5)
    f3, = ax.plot([], [], '-g', lw = 0.5)
    f4, = ax.plot([], [], '-g', lw = 0.5)
    f5, = ax.plot([], [], '-g', lw = 0.5)
    f6, = ax.plot([], [], '-k', lw = 0.5)
    f7, = ax.plot([], [], '-m', lw = 0.5)
    f8, = ax.plot([], [], '-m', lw = 0.5)
    f9, = ax.plot([], [], '-m', lw = 0.5)
    f10, = ax.plot([], [], '-m', lw = 0.5)
    f11, = ax.plot([], [], '-m', lw = 2)

    fs = [f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11]  # リスト化

    od, = ax.plot([], [], '+k', lw = 1)  # 移動障害物

    ## 計量
    metA0 = patches.Ellipse(xy = (state_history[0, 12], state_history[0, 13]), width = met[0, 1], height = met[0, 2], angle = met[0, 3], fill = False, color = 'r', lw = 2)
    metA1 = patches.Ellipse(xy = (state_history[0, 14], state_history[0, 15]), width = met[0, 4], height = met[0, 5], angle = met[0, 6], fill = False, color = 'g', lw = 2)
    metA2 = patches.Ellipse(xy = (state_history[0, 16], state_history[0, 17]), width = met[0, 7], height = met[0, 8], angle = met[0, 9], fill = False, color = 'g', lw = 0.5)
    metA3 = patches.Ellipse(xy = (state_history[0, 18], state_history[0, 19]), width = met[0, 10], height = met[0, 11], angle = met[0, 12], fill = False, color = 'g', lw = 0.5)
    metA4 = patches.Ellipse(xy = (state_history[0, 20], state_history[0, 21]), width = met[0, 13], height = met[0, 14], angle = met[0, 15], fill = False, color = 'g', lw = 0.5)
    metA5 = patches.Ellipse(xy = (state_history[0, 22], state_history[0, 23]), width = met[0, 16], height = met[0, 17], angle = met[0, 18], fill = False, color = 'g', lw = 0.5)
    metA6 = patches.Ellipse(xy = (state_history[0, 24], state_history[0, 25]), width = met[0, 19], height = met[0, 20], angle = met[0, 21], fill = False, color = 'k', lw = 0.5)
    metA7 = patches.Ellipse(xy = (state_history[0, 26], state_history[0, 27]), width = met[0, 22], height = met[0, 23], angle = met[0, 24], fill = False, color = 'm', lw = 0.5)
    metA8 = patches.Ellipse(xy = (state_history[0, 28], state_history[0, 29]), width = met[0, 25], height = met[0, 26], angle = met[0, 27], fill = False, color = 'm', lw = 0.5)
    metA9 = patches.Ellipse(xy = (state_history[0, 30], state_history[0, 31]), width = met[0, 28], height = met[0, 29], angle = met[0, 30], fill = False, color = 'm', lw = 0.5)
    metA10 = patches.Ellipse(xy = (state_history[0, 32], state_history[0, 33]), width = met[0, 31], height = met[0, 32], angle = met[0, 33], fill = False, color = 'm', lw = 0.5)
    metA11 = patches.Ellipse(xy = (state_history[0, 34], state_history[0, 35]), width = met[0, 34], height = met[0, 35], angle = met[0, 36], fill = False, color = 'm', lw = 2)

    metAs = [metA0, metA1, metA2, metA3, metA4, metA5, metA6, metA7, metA8, metA9, metA10, metA11]  # リスト化

    for i in np.arange(0, 12, 1):
        ax.add_patch(metAs[i])


    # 時刻表示
    time_template = 'time = %s'
    time_text = ax.text(0.05, 0.9, '', transform = ax.transAxes)


    def animate(i):
        """アニメーションの関数"""
        center.set_data(state_history[0:i, 1].T, state_history[0:i, 2].T)

        obs_history_u = obs_history[i:i+1, :].reshape(int(obs_history[i:i+1, :].size/2), 2)
        od.set_data(obs_history_u[:, 0], obs_history_u[:, 1])

        met_scale = 2
        for k in np.arange(0, 12, 1):  # 制御点毎に位置，計量，加速度を描写
            cpoints[k].set_data(state_history[i:i+1, 12 + 2 * k], state_history[i:i+1, 13 + 2 * k])
            metAs[k].set_center([state_history[i, 12+2*k], state_history[i, 13+2*k]])
            metAs[k].width = met[i, 3* k + 1] * met_scale
            metAs[k].height = met[i, 3* k + 2] * met_scale
            metAs[k].angle = met[i, 3*k + 3]
            fs[k].set_data(fvec[2*i:2*i+2, 2*k+1:2*k+2], fvec[2*i:2*i+2, 2*k+2:2*k+3])

        time_text.set_text(time_template % (i * time_interval))

        return center, cpoints[0], cpoints[1], cpoints[2], cpoints[3], cpoints[4], cpoints[5], cpoints[6], cpoints[7], cpoints[8], cpoints[9], cpoints[10], cpoints[11], \
            fs[0], fs[1], fs[2], fs[3], fs[4], fs[5], fs[6], fs[7], fs[8], fs[9], fs[10], fs[11], od, time_text, \
            metAs[0], metAs[1], metAs[2], metAs[3], metAs[4], metAs[5], metAs[6], metAs[7], metAs[8], metAs[9], metAs[10], metAs[11]


    ani = anm.FuncAnimation(fig = fig_ani,
                            func = animate,
                            frames = np.arange(0, int(tend/time_interval), 10),  # 枚数．一枚毎にするとクソ遅い．
                            init_func = None,
                            fargs = None,
                            interval = time_interval * 10e-3,  # コマ送り間隔[ms]
                            blit = True)
    ani_name = header + "__Trajectory.gif"
    ani.save(filename = save_path / ani_name, 
             fps = 1 / time_interval, 
             writer='pillow')

    print("アニメ化完了")
    print("アニメ化時間", time.time() - Time_ani_start)

    plt.show()


if __name__ == "__main__":
    mainsimu()