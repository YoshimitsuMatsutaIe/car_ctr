"""横滑りを考慮した車両ダイナミクス
・長谷川さんの卒論を丸コピ
・RMPシミュレーションで横滑りの外乱に対しロバストであることを示すため使用

仮定１：速度は十分０より大きい


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
import pathlib
from scipy.integrate import odeint


def carsimu(q, dqdt, v_command, beta_slip):
    """scipyで車両の運動方程式を解く
    
    ・滑り角，操舵角が微小なら使用可能

    input：

    return： numpy([[x, y, theta, slip_amgle, dthetadt]])
    
    """

    def car_eq(state, t, v_for, steer_angle):
        """車両の連立運動方程式
        input1：state（x,y,theta,slip_angle,omegaの初期値のタプル）
        imput2：時間列
        input3：
        
        """
        ## 定数と見做すパラメータ
        # 基本変えない（元論文のもの）
        #lf = 0.9560  # 正面から重心までの距離[m]
        #lr = 1.434  # 後ろから重心までの距離[m]
        lf = 0.2
        lr = 0.2

        m = 0.1640  # 車両質量[kg]
        I = 0.01093 # 車両の重心回りの慣性モーメント[sec^2/rad]
        Kf = 0.01  # コーナリングパワー係数[N/rad]．仮定：スリップ角が微笑
        Kr = 0.01  # コーナリングパワー係数[N/rad]．仮定：スリップ角が微笑

        v = v_for  # 車両並進速度．制御ループ間一定と見做す．
        delta = steer_angle  # 操舵角．制御ループ間一定と見做す．


        # 状態変数
        x = state[0]
        y = state[1]
        theta = state[2]
        slip_angle = state[3]
        omega = state[4]

        # 状態変数の微分
        dxdt = v * cos(theta + slip_angle)
        dydt = v * sin(theta + slip_angle)
        dthetadt = omega
        dslip_angledt = 1 / (m * v) * ((-Kf * (slip_angle - delta) * cos(delta) - Kr * slip_angle) \
            - (m * v + (Kf * lf / v * cos(delta)) - (Kr * lr / v)) * omega)
        domegadt = 1 / I * (-lf * Kf * (slip_angle + lf * omega / v - delta) * cos(delta) + lr * Kr * (slip_angle - lr * omega / v))

        return ([dxdt, dydt, dthetadt, dslip_angledt, domegadt])

    times = np.arange(0, 0.1, 0.01, dtype = np.float32)  # 制御ループ時間だけ計算する．
    init_state = [q[0, 0], q[1, 0], q[2, 0], beta_slip[0, 0], dqdt[2, 0]]  # ここからスタート

    traj = odeint(func = car_eq,
                  y0 = init_state,
                  t = times, 
                  args = (v_command[0, 0], v_command[1, 0]))
    #print(traj.shape)
    #print(traj)

    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    #ax.plot(traj[:, 0], traj[:, 1])
    #ax.set_aspect('equal')
    #ax.grid(True)
    #ax.set_xlabel('X')
    #ax.set_ylabel('Y')
    #plt.show()

    end = traj.shape[0]

    return traj[end-1:end, :]  # 最後だけ返す



def carsimu_2(q, dqdt, v_command, beta_slip):
    """scipyで車両の運動方程式を解く
    
    ・滑り角，操舵角が微小じゃなくても使用可能？
    ・速度が小さいと発散
    https://myenigma.hatenablog.com/entry/2017/05/14/151539#%E7%9B%AE%E6%A8%99%E3%82%B3%E3%83%BC%E3%82%B9%E5%BA%A7%E6%A8%99%E7%B3%BB%E3%81%AB%E3%81%8A%E3%81%91%E3%82%8B%E7%B7%9A%E5%BD%A2Dynamic-Bicycle-Model
    input：

    return： numpy([[x, y, theta, slip_amgle, dthetadt]])
    
    """

    def car_eq_2(state, t, v_for, steer_angle):
        """車両の連立運動方程式
        input1：state（x,y,theta,slip_angle,omegaの初期値のタプル）
        imput2：時間列
        input3：
        
        """
        ## 定数と見做すパラメータ
        # 基本変えない（元論文のもの）
        #lf = 0.9560  # 正面から重心までの距離[m]
        #lr = 1.434  # 後ろから重心までの距離[m]
        lf = 0.2
        lr = 0.2

        m = 0.1640  # 車両質量[kg]
        I = 0.01093 # 車両の重心回りの慣性モーメント[sec^2/rad]
        Cf = 0.01  # コーナリング（スティフネス）パワー係数[N/rad]．仮定：スリップ角が微笑
        Cr = 0.01  # コーナリング（スティフネス）パワー係数[N/rad]．仮定：スリップ角が微笑

        a = v_for / 0.1  # 車両加速度．制御ループ間一定と見做す．
        delta = steer_angle  # 操舵角．制御ループ間一定と見做す．


        # 状態変数
        x = state[0]
        y = state[1]
        theta = state[2]
        vx = state[3]
        vy = state[4]
        omega = state[5]

        #Ffy = -Cf * ((vy + lf * omega) / vx - delta)
        #Fry = -Cr * ((vy - lr * omega) / vx)

        # 状態変数の微分
        dxdt = vx * cos(theta) - vy * sin(theta)
        dydt = vx * sin(theta) - vy * cos(theta)
        dthetadt = omega
        dvxdt = a - ((-Cf * ((vy + lf * omega) / vx - delta)) * sin(delta) / m) + vy * omega
        dvydt = (-Cr * ((vy - lr * omega) / vx)) / m + ((-Cf * ((vy + lf * omega) / vx - delta)) * cos(delta) / m) - vx * omega
        domegadt = ((-Cf * ((vy + lf * omega) / vx - delta)) * lf * cos(delta) - (-Cr * ((vy - lr * omega) / vx)) * lr )/ I

        return ([dxdt, dydt, dthetadt, dvxdt, dvydt, domegadt])

    times = np.arange(0, 0.1, 0.01, dtype = np.float32)  # 制御ループ時間だけ計算する．

    vx_init = v_command[0, 0] * cos(beta_slip)
    vy_init = v_command[0, 0] * sin(beta_slip)

    init_state = [q[0, 0], q[1, 0], q[2, 0], vx_init, vy_init, dqdt[2, 0]]  # ここからスタート

    traj = odeint(func = car_eq_2,
                  y0 = init_state,
                  t = times, 
                  args = (v_command[0, 0], v_command[1, 0]))
    

    #print(traj.shape)
    #print(traj)

    #slip = np.arctan2(traj[:, 4].T, traj[:, 3].T)

    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    #ax.plot(times, traj[:, 0], label = "x")
    #ax.plot(times, traj[:, 1], label = "y")
    #ax.plot(times, traj[:, 2], label = "theta")
    #ax.plot(times, traj[:, 3], label = "vx")
    #ax.plot(times, traj[:, 4], label = "vy")
    #ax.plot(times, traj[:, 5], label = "omega")
    #ax.plot(times, slip, label = "slip")
    ##ax.set_aspect('equal')
    #ax.grid(True)
    #ax.legend(loc = 'best')
    #ax.set_xlabel('t')
    #ax.set_ylabel('every')


    #figXY = plt.figure()
    #axXY = figXY.add_subplot(111)
    #axXY.plot(traj[:, 0], traj[:, 1], label = "trajectory")
    #axXY.set_aspect('equal')
    #axXY.grid(True)
    #axXY.legend(loc = 'best')
    #axXY.set_xlabel('X')
    #axXY.set_ylabel('Y')

    #plt.show()

    end = traj.shape[0]
    slip_angle_new = np.array([[np.arctan2(traj[end-1, 4], traj[end-1, 3])]], np.float32)
    q_new = traj[end-1:end, 0:3].T
    dqdt_new = np.array([[v_command[0, 0] * cos(q_new[2, 0] + slip_angle_new)],
                         [v_command[0, 0] * sin(q_new[2, 0] + slip_angle_new)],
                         [traj[end-1, 5]]], np.float32)

    return q_new, dqdt_new, slip_angle_new



if __name__ == "__main__":
    carsimu()
    carsimu_2()
