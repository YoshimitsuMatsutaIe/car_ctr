"""グラフ作成ツール"""

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
from matplotlib.font_manager import FontProperties
fp = FontProperties(fname=r'C:\WINDOWS\Fonts\Arial.ttf', size=30)

def make_youshi_fig(path_conventional, path_proposed, path_obs):
    """要旨に使う図を作成"""

    save_path = pathlib.Path(r'D:\control_study_2020\RMPver18\卒論\提出版卒論に使用')  # 家
    #save_path = pathlib.Path(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用')  # 大学


    state_history_con = np.loadtxt(path_conventional, delimiter=',', dtype='float32')
    state_history_pro = np.loadtxt(path_proposed, delimiter=',', dtype='float32')
    obs_history = np.loadtxt(path_obs, delimiter=',', dtype='float32')

    print("データ読み込み完了")
    posi_g = np.array([[10, 0.5]])
    
    ## グラフ化 ###
    # グラフサイズ決定
    posi_all = np.concatenate([posi_g, state_history_con[:, 1:3], state_history_pro[:, 1:3]], axis = 0)
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

    fig_W = 8
    fig_L = 8

    obs_init = obs_history[0:1, :].reshape(int(obs_history[0:1, :].size/2), 2)

    print("グラフ化中．．．")

    ##軌跡（グラデーション付き）
    fig_Tra, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))

    ax.scatter(posi_g[0, 0], posi_g[0, 1], 
               s = 350, label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')
    ax.scatter(obs_init[:, 0], obs_init[:, 1], 
               s = 200, label = 'obstacle points', marker = '+', color = 'k', alpha = 1, linewidths = 1)
    ax.scatter(0, 0,
               s = 100, label = 'start point', marker = ',', color = 'k', alpha = 1)

    ## グラデーション付き
    #v_max = np.amax(np.concatenate([state_history_con[:, 7], state_history_pro[:, 7]], axis = 0))
    #v_min = np.amin(np.concatenate([state_history_con[:, 7], state_history_pro[:, 7]], axis = 0))
    #for i in np.arange(0,637, 1):
    #    ax.plot(state_history_con[i:i+2, 1].T, 
    #            state_history_con[i:i+2, 2].T, 
    #            color = cm.winter((state_history_con[i, 7] - v_min) / (v_max - v_min)), 
    #            markersize = 1.5)
    #    ax.plot(state_history_pro[i:i+2, 1].T, 
    #            state_history_pro[i:i+2, 2].T, 
    #            color = cm.winter((state_history_pro[i, 7] - v_min) / (v_max - v_min)), 
    #            markersize = 1.5)

    # グラデーション無し
    ax.plot(state_history_con[:, 1].T, 
            state_history_con[:, 2].T, 
            label = 'Conventional', linestyle = 'dashed', color = 'k', linewidth = 2.0)
    ax.plot(state_history_pro[:, 1].T, 
            state_history_pro[:, 2].T, 
            label = 'Proposed', linestyle = 'solid', color = 'k', linewidth = 2.0)



    ax.set_xlabel('$\it{X(m)}$', fontsize = 10)
    ax.set_ylabel('$\it{Y(m)}$', fontsize = 10)
    ax.grid(True)

    ## データより自動で決定
    #ax.set_xlim(fig_xmin - 1, fig_xmax + 1)  
    #ax.set_ylim(fig_ymin - 1, fig_ymax + 1)

    # 手動
    ax.set_xlim(-1, 12)  
    ax.set_ylim(-1.7, 4.3)

    ax.legend(loc='best', fontsize = 10)
    ax.set_aspect('equal')

    fig_Tra_name = "重ね合わせ動的.png"
    fig_Tra.savefig(save_path / fig_Tra_name)

    plt.show()


def make_soturon_fig():
    """卒論の実験結果グラフを作成"""
    
    #save_path = pathlib.Path(r'D:\control2020\RMP_exp\Exp_data_home\2021_01_27')  # 家
    save_path = pathlib.Path(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用')  # 大学



    path_conventional = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__提案手法ためし動的2021-02-15--06-56-55__State_history_temp.csv'

    path_obs = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__従来手法ためし静的2021-02-15--06-19-55__obs_history_temp.csv'
    path_proposed = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__従来手法ためし静的2021-02-15--06-19-55__State_history_temp.csv'

    state_history_con = np.loadtxt(path_conventional, delimiter=',', dtype='float32')
    state_history_pro = np.loadtxt(path_proposed, delimiter=',', dtype='float32')
    obs_history = np.loadtxt(path_obs, delimiter=',', dtype='float32')

    print("データ読み込み完了")
    posi_g = np.array([[10, 0.5]])

        ## グラフ化 ###
    # グラフサイズ決定

    #fig_xmin = -1
    #fig_xmax = 11
    #fig_ymin = -2
    #fig_ymax = 6


    #fig_max_length = 13

    #x_scale = fig_xmax - fig_xmin
    #y_scale = fig_ymax - fig_ymin

    #if x_scale >= y_scale:
    #    fig_W = fig_max_length
    #    fig_L = fig_max_length * y_scale / x_scale
    #else:
    #    fig_W = fig_max_length * x_scale / y_scale
    #    fig_L = fig_max_length

    fig_W = 15
    fig_L = 10

    obs_init = obs_history[9:10, :].reshape(int(obs_history[9:10, :].size/2), 2)

    print("グラフ化中．．．")

    ##軌跡（グラデーション付き） 
    plt.rcParams["font.size"] = 30
    fig_Tra, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))

    ax.scatter(posi_g[0, 0], posi_g[0, 1], 
               s = 800, label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')
    ax.scatter(obs_init[:, 0], obs_init[:, 1], 
               s = 400, label = 'obstacle points', marker = '+', color = 'k', alpha = 1, linewidths = 1)
    ax.scatter(0, 0,
               s = 200, label = 'start point', marker = ',', color = 'k', alpha = 1)

    #ax.scatter(posi_g[0, 0], posi_g[0, 1], 
    #           label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')
    #ax.scatter(obs_init[:, 0], obs_init[:, 1], 
    #           label = 'obstacle points', marker = '+', color = 'k', alpha = 1, linewidths = 1)
    #ax.scatter(0, 0,
    #           label = 'start point', marker = ',', color = 'k', alpha = 1)



    ## グラデーション付き
    #v_max = np.amax(np.concatenate([state_history_con[:, 7], state_history_pro[:, 7]], axis = 0))
    #v_min = np.amin(np.concatenate([state_history_con[:, 7], state_history_pro[:, 7]], axis = 0))
    #for i in np.arange(0, state_history_con.shape[0], 1):
    #    ax.plot(state_history_con[i:i+2, 1].T, 
    #            state_history_con[i:i+2, 2].T, 
    #            color = cm.winter((state_history_con[i, 7] - v_min) / (v_max - v_min)), 
    #            linewidth = 5)
        #ax.plot(state_history_pro[i:i+2, 1].T, 
        #        state_history_pro[i:i+2, 2].T, 
        #        color = cm.winter((state_history_pro[i, 7] - v_min) / (v_max - v_min)), 
        #        markersize = 1.5)

    ## グラデーション無し
    #ax.plot(state_history_con[:, 1].T, 
    #        state_history_con[:, 2].T, 
    #        label = 'Conventional', linestyle = 'dashed', color = 'k', linewidth = 2.0)
    #ax.plot(state_history_pro[:, 1].T, 
    #        state_history_pro[:, 2].T, 
    #        label = 'Proposed', linestyle = 'solid', color = 'k', linewidth = 2.0)

    

    #ax.set_xlabel('$\it{X}$(m)', fontsize = 30)
    #ax.set_ylabel('$\it{Y}$(m)', fontsize = 30)
    ax.set_xlabel('$\it{X}$(m)')
    ax.set_ylabel('$\it{Y}$(m)')
    ax.grid(True)

    ## データより自動で決定
    #ax.set_xlim(fig_xmin - 1, fig_xmax + 1)  
    #ax.set_ylim(fig_ymin - 1, fig_ymax + 1)

    # 手動
    ax.set_xlim(-1, 12)  
    ax.set_ylim(-2, 6)

    #ax.legend(loc='best', fontsize = 30)
    ax.legend(loc='best')
    ax.set_aspect('equal')

    fig_Tra_name = "実験環境.png"
    fig_Tra.savefig(save_path / fig_Tra_name)

    plt.show()


def make_soturon_fig_iroiro():
    """卒論の実験結果グラフ（状態，入力）を作成"""
    
    #save_path = pathlib.Path(r'D:\control2020\RMP_exp\Exp_data_home\2021_01_27')  # 家
    save_path = pathlib.Path(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用')  # 大学



    #path = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__提案手法ためし動的2021-02-15--06-56-55__State_history_temp.csv'
    #path = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\timeover__提案手法ためし静的2021-02-15--06-24-43__State_history_temp.csv'
    #path = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\collision__従来手法ためし動的2021-02-15--06-55-17__State_history_temp.csv'
    path = r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__従来手法ためし静的2021-02-15--06-19-55__State_history_temp.csv'




    state_history = np.loadtxt(path, delimiter=',', dtype='float32')

    print(state_history.shape)

    tend = state_history[state_history.shape[0]-1, 0]

    print("データ読み込み完了")


    fig_W = 20
    fig_L = 13


    print("グラフ化中．．．")

    comment = "従来静的"

    plt.rcParams["font.size"] = 30

    fig_x, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 1], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel('Position $\it{x}$ [m]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_x_name = comment + "x.png"
    fig_x.savefig(save_path / fig_x_name)


    fig_y, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 2], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel('Position $\it{y}$ [m]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_y_name = comment + "y.png"
    fig_y.savefig(save_path / fig_y_name)

    fig_theta, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 3], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel(r'Posture $\theta$ [rad]')
    ax.grid(True)
    ax.set_xlim(0, tend)
    #ax.set_ylim(-pi, pi)
    fig_theta_name = comment + "theta.png"
    fig_theta.savefig(save_path / fig_theta_name)

    fig_dxdt, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 4], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel('Velocity-x $\.{x}$ [m/s]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_dxdt_name = comment + "dxdt.png"
    fig_dxdt.savefig(save_path / fig_dxdt_name)

    fig_dydt, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 5], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel('Velocity-y $\.{y}$ [m/s]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_dydt_name = comment + "dydt.png"
    fig_dydt.savefig(save_path / fig_dydt_name)

    fig_dthetadt, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 6], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel(r'Angular Velocity $\.{θ}$ [rad/s]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_dthetadt_name = comment + "dthetadt.png"
    fig_dthetadt.savefig(save_path / fig_dthetadt_name)


    fig_v, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 7], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel('Forward Velocity $\it{v}$ [m/s]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_v_name = comment + "v.png"
    fig_v.savefig(save_path / fig_v_name)

    fig_xi, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 8], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel(r'Steering Angle $\it{ξ}$ [rad]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_xi_name = comment + "xi.png"
    fig_xi.savefig(save_path / fig_xi_name)

    fig_dvdt, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 10], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel(r'Forward Acceleration $\.{v} [m/s^{2}]$')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_dvdt_name = comment + "dvdt.png"
    fig_dvdt.savefig(save_path / fig_dvdt_name)

    fig_dxidt, ax = plt.subplots(nrows = 1, ncols = 1, figsize = (fig_W, fig_L))
    ax.plot(state_history[:, 0], state_history[:, 11], linewidth = 5)
    ax.set_xlabel('Time $\it{t}$ [s]')
    ax.set_ylabel(r'Steering Angular Velocity $\.{ξ}$ [rad/s]')
    ax.grid(True)
    ax.set_xlim(0, tend)  
    #ax.set_ylim(-2, 6)
    fig_dxidt_name = comment + "dxidt.png"
    fig_dxidt.savefig(save_path / fig_dxidt_name)


def make_soturon_gif():
    save_path = pathlib.Path(r'D:\control_study_2020\RMPver18\卒論\提出版卒論に使用')  # 家
    #save_path = pathlib.Path(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用')  # 大学



    path_conventional = r'D:\control_study_2020\RMPver18\卒論\提出版卒論に使用\collision__従来手法ためし動的2021-02-15--06-55-17__State_history_temp.csv'

    path_obs = r'D:\control_study_2020\RMPver18\卒論\提出版卒論に使用\succes__提案手法ためし動的2021-02-15--06-56-55__obs_history_temp.csv'
    path_proposed = r'D:\control_study_2020\RMPver18\卒論\提出版卒論に使用\succes__提案手法ためし動的2021-02-15--06-56-55__State_history_temp.csv'

    state_history_con = np.loadtxt(path_conventional, delimiter=',', dtype='float32')
    state_history_pro = np.loadtxt(path_proposed, delimiter=',', dtype='float32')
    obs_history = np.loadtxt(path_obs, delimiter=',', dtype='float32')

    print("データ読み込み完了")
    posi_g = np.array([[10, 0.5]])

    fig_W = 15
    fig_L = 10

    obs_init = obs_history[9:10, :].reshape(int(obs_history[9:10, :].size/2), 2)

    print("グラフ化中．．．")

    plt.rcParams["font.size"] = 30
    fig_ani = plt.figure(figsize = (fig_W, fig_L))
    ax = fig_ani.add_subplot(111, xlim=(-1, 12), ylim=(-2, 4))
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    #ax.legend("best")

    ax.scatter(posi_g[0,0], posi_g[0,1], 
               s = 300, label = 'goal point', marker = '*', color = '#ff7f00', alpha = 1, linewidths = 1.5, edgecolors = 'red')



    center, = ax.plot([], [], '-m', lw=2)  # 中心軌跡

    # 以下制御点
    cpoint0, = ax.plot([], [], '.-.r', lw=3)  # 鼻先
    cpoint1, = ax.plot([], [], '.-.k', lw=1)
    cpoint2, = ax.plot([], [], '.-.k', lw=1)
    cpoint3, = ax.plot([], [], '.-.k', lw=1)
    cpoint4, = ax.plot([], [], '.-.k', lw=1)
    cpoint5, = ax.plot([], [], '.-.k', lw=1)
    cpoint6, = ax.plot([], [], '.-.k', lw=1)
    cpoint7, = ax.plot([], [], '.-.k', lw=1)
    cpoint8, = ax.plot([], [], '.-.k', lw=1)
    cpoint9, = ax.plot([], [], '.-.k', lw=1)
    cpoint10, = ax.plot([], [], '.-.k', lw=1)
    cpoint11, = ax.plot([], [], '.-.k', lw=1)

    cpoints = [cpoint0, cpoint1, cpoint2, cpoint3, cpoint4, cpoint5, cpoint6, cpoint7, cpoint8, cpoint9, cpoint10, cpoint11]  # リスト化

    c2enter, = ax.plot([], [], '-b', lw=2)  # 中心軌跡

    # 以下制御点
    c2point0, = ax.plot([], [], '.-.r', lw=3)  # 鼻先
    c2point1, = ax.plot([], [], '.-.k', lw=1)
    c2point2, = ax.plot([], [], '.-.k', lw=1)
    c2point3, = ax.plot([], [], '.-.k', lw=1)
    c2point4, = ax.plot([], [], '.-.k', lw=1)
    c2point5, = ax.plot([], [], '.-.k', lw=1)
    c2point6, = ax.plot([], [], '.-.k', lw=1)
    c2point7, = ax.plot([], [], '.-.k', lw=1)
    c2point8, = ax.plot([], [], '.-.k', lw=1)
    c2point9, = ax.plot([], [], '.-.k', lw=1)
    c2point10, = ax.plot([], [], '.-.k', lw=1)
    c2point11, = ax.plot([], [], '.-.k', lw=1)

    c2points = [c2point0, c2point1, c2point2, c2point3, c2point4, c2point5, c2point6, c2point7, c2point8, c2point9, c2point10, c2point11]  # リスト化



    od, = ax.plot([], [], '+k', lw = 3)  # 移動障害物



    # 時刻表示
    time_template = 'time = %s' + '[s]'
    time_text = ax.text(0.05, 0.9, '', transform = ax.transAxes)

    def animate_exist_move_obs(i):
        """アニメーションの関数（動的障害物あり）"""
        center.set_data(state_history_con[0:i, 1].T, state_history_con[0:i, 2].T)
        c2enter.set_data(state_history_pro[0:i, 1].T, state_history_pro[0:i, 2].T)

        obs_history_u = obs_history[i:i+1, :].reshape(int(obs_history[i:i+1, :].size/2), 2)
        od.set_data(obs_history_u[:, 0], obs_history_u[:, 1])

        for k in np.arange(0, 12, 1):  # 制御点毎に位置を描写
            cpoints[k].set_data(state_history_con[i:i+1, 12 + 2 * k], state_history_con[i:i+1, 13 + 2 * k])
            c2points[k].set_data(state_history_pro[i:i+1, 12 + 2 * k], state_history_pro[i:i+1, 13 + 2 * k])

        time_text.set_text(time_template % (i * 0.1))

        return center, cpoints[0], cpoints[1], cpoints[2], cpoints[3], cpoints[4], cpoints[5], cpoints[6], cpoints[7], cpoints[8], cpoints[9], cpoints[10], cpoints[11], od, time_text,\
            c2enter, c2points[0], c2points[1], c2points[2], c2points[3], c2points[4], c2points[5], c2points[6], c2points[7], c2points[8], c2points[9], c2points[10], c2points[11]
           


    ani = anm.FuncAnimation(fig = fig_ani,
                            func = animate_exist_move_obs,
                            frames = np.arange(0, 790, 10),  # 枚数．一枚毎にするとクソ遅い．
                            init_func = None,
                            fargs = None,
                            interval = 0.1 * 10e-3,  # コマ送り間隔[ms]
                            blit = True)

    ani_name = "スライド用.gif"
    ani.save(filename = save_path / ani_name, 
             fps = 1 / 0.1, 
             writer='pillow')

    print("アニメ化完了")


    plt.show()

