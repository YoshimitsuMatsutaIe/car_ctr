"""RMPで最適加速度を計算する関数群
・元論文のやつ
・マニピュレータのやつ
・自分で考えた？やつ
の3つ
．RMPflowはない？
"""

import numpy as np
from math_utils import *  # いろんな関数のモジュール


## マニピュレータの論文[R1]のやつ
def soft_normal(v, alpha):
    """ソフト正規化関数"""
    v_norm = np.linalg.norm(v)
    softmax = v_norm + 1 / alpha * np.log(1 + np.exp(-2 * alpha * v_norm))
    return v / softmax

def A_stretch(v, alpha):
    """空間を一方向に伸ばす計量"""
    xi = soft_normal(v, alpha)
    return xi @ xi.T

def basic_metric_H(f, alpha, beta):
    """基本の計量"""
    f_norm = np.linalg.norm(f)
    f_softmax = f_norm + 1 / alpha * np.log(1 + np.exp(-2 * alpha * f_norm))
    s = f / f_softmax
    return beta * s @ s.T + (1 - beta) * np.eye(2)

def f_attract(z, dzdt, z0, i):
    """アトラクタ加速度"""
    
    # パラメータ
    max_speed = 0.5
    if i == 0:
        gain = 1
        damp = gain / max_speed
    if i == 1 or i == 11:
        gain = 5
        damp = gain / max_speed
    alpha = 2  # 減速半径

    #y = -np.exp(-np.linalg.norm(z0 - z)/2) * 1 / 1 * (z0 - z)
    #carv = -1/2 * np.linalg.norm(dzdt) ** 2 * (np.eye(2) - 2 * dzdt @ dzdt.T) @ y
    carv = 0

    return gain * soft_normal(z0 - z, alpha) - damp * dzdt -carv

def A_attract(z, dzdt, z0, f_attract, i):
    """アトラクタ計量"""
    
    # パラメータ
    sigma_W = 10
    sigma_H = 1
    alpha = 2  # 減速半径
    dis = np.linalg.norm(z0 - z)
    weight = np.exp(-dis / sigma_W)
    beta_attract = 1 - np.exp(-1 / 2 * (dis / sigma_H) ** 2)
    #beta_attract = 0
    #print(beta_attract)
    #print(weight * basic_metric_H(f_attract, alpha, beta_attract))
    return weight * basic_metric_H(f_attract, alpha, beta_attract) # 論文
    #return np.eye(2, dtype = np.float32)  # 提案

def f_obs(z, dzdt, z0, i):
    """障害物加速度"""

    scale_rep = 1.5  # 感知半径を表す？
    scale_damp = 1


    ratio = 0.5

    if i == 0 or i == 1 or i == 11:  # 先頭３つ
        rep_gain = 1
        damp_gain = rep_gain * ratio
    elif 1 < i < 5 or 7 < i < 11:  # 左右側面３つずつ
        rep_gain = 0.01
        damp_gain = rep_gain * ratio
    else:  # ケツ3つ
        rep_gain = 0.00001
        damp_gain = rep_gain * ratio

    #x = z0 - z  # 反対かも？
    x = z - z0
    #dzdt = -dzdt  # 二乗するから関係なし
    #dodt = np.array([[-0.05, 0]], np.float32).T
    #dzdt = dodt - dzdt

    dis = np.linalg.norm(x)  # 距離関数
    dis_grad = x / dis  # 距離関数の勾配

    #dis = 1 / 2 * (np.linalg.norm(x))**2  # 距離関数
    #dis_grad = x / np.linalg.norm(x)  # 距離関数の勾配


    # 斥力項．障害物に対する位置ベースの反発力？
    alpha_rep = rep_gain * np.exp(-dis / scale_rep)  # 斥力の活性化関数
    f_rep = alpha_rep * dis_grad  # 斥力項

    # ダンピング項．障害物に向かう速度にペナルティを課す？
    P_obs = max(0, -(dzdt).T @ dis_grad) * dis_grad @ dis_grad.T @ dzdt  # 零空間射影演算子
    alpha_damp = damp_gain / (dis / scale_damp + 1e-7)  # ダンピングの活性化関数
    f_damp = alpha_damp * P_obs  # ダンピング項

    #print(f_rep + f_damp)

    return f_rep + f_damp

def A_obs(z, dzdt, z0, f_obs, i):
    """障害物計量"""
    r = 15  # この半径外に障害物が来ると発散
    x = z - z0
    dis = np.linalg.norm(x)
    weight_obs = (dis / r) ** 2 - 2 * dis / r + 1  #3次スプライン?

    #weight_obs = 1 / (1 + np.exp(1.1 * dis - 5))  # シグモイド

    #multi = 1
    #coeff = 1
    #offset = 0
    #weight_obs = multi * np.exp(-(dis - offset) * coeff)  # 指数関数

    #r_react = 2  # 感知半径
    #b = 1
    #a = b / r_react
    #weight_obs = max(-a * dis + b, 0)
    #print(weight_obs * basic_metric_H(f_obs, 1, 1))

    #weight_obs = 0
    #return weight_obs * basic_metric_H(f_obs, 1, 0.8)  # これかも？
    return weight_obs * np.eye(2, dtype = np.float32)  # 誤植？
    ##print(f_obs @ f_obs.T)
    #return weight_obs * 100 * f_obs @ f_obs.T


# 自作RMP
def f_obs_move(xi_global, dxidt_global, posi_oj, uij, i):
    """自作の移動障害物加速度"""

    ## パラメータ

    dodt = np.array([[-0.05, 0]], np.float32).T

    if i == 0 or i == 1 or i == 11:  # 先頭３つ
        alpha_o = 1  # 障害物ゲイン
        beta_o = 1  # 障害物ダンピング
        gamma_o = 0 # 障害物オフセット
    elif 1 < i < 5 or 7 < i < 11:  # 左右側面３つずつ
        alpha_o = 0.01  # 障害物ゲイン
        beta_o = 0.01  # 障害物ダンピング
        gamma_o = 0.0 # 障害物オフセット
    else:  # ケツ3つ
        alpha_o = 0
        beta_o = 0
        gamma_o = 0

    w_new = np.exp(np.linalg.norm(dodt)*10)
    #w_new = 1
    return w_new * -alpha_o / np.linalg.norm(posi_oj - xi_global) * (beta_o * ((dodt - dxidt_global).T @ uij) + gamma_o) * uij  # 機械学会要旨
    #return -alpha_o / np.linalg.norm(posi_oj - xi_global) * (beta_o * (dodt/np.linalg.norm(dodt + 0.001)).T @ dxidt_global/np.linalg.norm(dxidt_global + 0.001)) * uij * 0



def A_obs_move(xi_global, posi_oj, foij):
    """元論文の障害物計量"""
    dis_obs = np.linalg.norm(posi_oj - xi_global)
    weight = expo(multi = 1, coeff = -1/10, x_offset = 0, x = dis_obs)
    #print(foij @ foij.T)
    return weight * foij @ foij.T
    #return 1000 * np.eye(2, dtype = np.float32)*0  #　なぜかこれだとうまくいく



# 元論文関係
def uij_cal(xi_global, posi_oj):
    """障害物への単位方向ベクトル"""
    return (posi_oj - xi_global) / np.linalg.norm(posi_oj - xi_global)

def fgi_cal(xi_global, dxidt_global, posi_g, i):
    """元論文の目標加速度"""

    # パラメータ
    max_speed = 0.5
    max_acc_g_head = 0.01
    max_acc_g_rl = 0.06
    alpha_g_head = max_acc_g_head
    alpha_g_rl = max_acc_g_rl
    beta_g_head = max_acc_g_head / max_speed
    beta_g_rl = max_acc_g_rl / max_speed
    epsi_g = 1 * 10e-7  # 目標到達時の発散防止
    if i == 0:
        alpha_g = alpha_g_head
        beta_g = beta_g_head
    else:
        alpha_g = alpha_g_rl
        beta_g = beta_g_rl

    #y = -np.exp(-np.linalg.norm(posi_g - xi_global)/2) * 1 / 1 * (posi_g - xi_global)
    #carv = -1/2 * np.linalg.norm(dxidt_global) ** 2 * (np.eye(2) - 2 * dxidt_global @ dxidt_global.T) @ y
    carv = 0

    return alpha_g * (posi_g - xi_global) / np.linalg.norm(posi_g - xi_global + epsi_g) - beta_g * dxidt_global - carv

def Agi_cal():
    """元論文の目標計量"""
    return np.eye(2, dtype = np.float32)
   
def foij_cal(xi_global, dxidt_global, posi_oj, uij, i):
    """元論文の障害物加速度"""

    ## パラメータ
    #alpha_o = 1  # 障害物ゲイン
    #beta_o = 1  # 障害物ダンピング
    #gamma_o = 0.2 # 障害物オフセット

    if i == 0 or i == 1 or i == 11:  # 先頭３つ
        alpha_o = 1  # 障害物ゲイン
        beta_o = 0.5  # 障害物ダンピング
        gamma_o = 0.3 # 障害物オフセット
    elif 1 < i < 5 or 7 < i < 11:  # 左右側面３つずつ
        alpha_o = 0.01  # 障害物ゲイン
        beta_o = 0.1  # 障害物ダンピング
        gamma_o = 0.2 # 障害物オフセット
    else:  # ケツ3つ
        alpha_o = 0.0001
        beta_o = 0.0001
        gamma_o = 0.0001



    return -alpha_o / np.linalg.norm(posi_oj - xi_global) * (beta_o * (dxidt_global.T @ uij) + gamma_o) * uij

def Aoij_cal(xi_global, posi_oj, foij):
    """元論文の障害物計量"""
    dis_obs = np.linalg.norm(posi_oj - xi_global)
    weight = expo(multi = 1, coeff = -1/2, x_offset = 0, x = dis_obs)
    return weight * foij @ foij.T
    #return weight * np.eye(2, dtype = np.float32)  #　なぜかこれだとうまくいく


def compute_cp_RMP_nonvec(xi_global, dxidt_global, posi_g, posi_o, jend, i):
    """ベクトル化なしで各制御点のRMPcを計算
    ・基本のやつ
    ・すごく遅い
    ・自作RMPの試験用またはデバッグ用
    """
    
    
    #print("i=",i)


    ## 元論文
    #if i == 0 or i == 1 or i == 11:
    #    fc_Right_term = fgi_cal(xi_global, dxidt_global, posi_g.T, i)
    #    fc_Left_term = Agi_cal()
    #else:
    #    fc_Right_term = np.zeros((2, 1), dtype = np.float32)
    #    fc_Left_term = np.zeros((2, 2), dtype = np.float32)

    #for j in np.arange(0, jend, 1):
    #    u = uij_cal(xi_global, posi_o[j:j + 1, :].T)
    #    fo = foij_cal(xi_global, dxidt_global, posi_o[j:j + 1, :].T, u, i)
    #    #fo = f_obs_move(xi_global, dxidt_global, posi_o[j:j + 1, :].T, u, i)
    #    Ao =  Aoij_cal(xi_global, posi_o[j:j + 1, :].T, fo)
    #    #Ao = np.eye(2)
    #    fc_Right_term  = fc_Right_term + Ao @ fo
    #    fc_Left_term = fc_Left_term + Ao


    ## マニピュレータ
    #if i == 0 or i == 1 or i == 11:
    #    fc_Right_term = f_attract(xi_global, dxidt_global, posi_g.T, i)
    #    fc_Left_term = A_attract(xi_global, dxidt_global, posi_g.T, fc_Right_term, i)
    #else:
    #    fc_Right_term = np.zeros((2, 1), dtype = np.float32)
    #    fc_Left_term = np.zeros((2, 2), dtype = np.float32)

    #for j in np.arange(0, jend, 1):
    #    fo = f_obs(xi_global, dxidt_global, posi_o[j:j + 1, :].T, i)
    #    Ao =  A_obs(xi_global, dxidt_global, posi_o[j:j+1, :].T, fo, i)
        
    #    #print(j)
    #    #print(Ao @ fo)
    #    #print(Ao)
    #    fc_Right_term  = fc_Right_term + Ao @ fo
    #    fc_Left_term = fc_Left_term + Ao


    # 自作＋マニピュレータ
    if i == 0 or i == 1 or i == 11:
        fc_Right_term = f_attract(xi_global, dxidt_global, posi_g.T, i)
        fc_Left_term = A_attract(xi_global, dxidt_global, posi_g.T, fc_Right_term, i)
    else:
        fc_Right_term = np.zeros((2, 1), dtype = np.float32)
        fc_Left_term = np.zeros((2, 2), dtype = np.float32)

    for j in np.arange(0, jend, 1):
        fo = f_obs(xi_global, dxidt_global, posi_o[j:j + 1, :].T, i)
        Ao =  A_obs(xi_global, dxidt_global, posi_o[j:j+1, :].T, fo, i)
        #fo = np.zeros((2, 1))
        #ao = np.zeros((2,2))
        u = uij_cal(xi_global, posi_o[j:j + 1, :].T)
        fo_move = f_obs_move(xi_global, dxidt_global, posi_o[j:j + 1, :].T, u, i)
        Ao_move = A_obs_move(xi_global, posi_o[j:j + 1, :].T, fo_move)
        #print("f_obs=", fo)
        #print("a_obs=", ao)
        #print("f_obs_move=", fo_move)
        #print("a_obs_move=", ao_move)
        #fo_move = np.zeros((2,1), np.float32)
        #Ao_move = np.zeros((2,2), np.float32)
        fc_Right_term += Ao @ fo + Ao_move @ fo_move
        fc_Left_term += Ao + Ao_move

    ##if np.linalg.norm(fc_Left_term) < 1e-3:  # 発散防止
    ##    fc_Left_term = np.zeros((2, 2))

    fc = np.linalg.pinv(fc_Left_term) @ fc_Right_term
    Ac = fc_Left_term

    
    #print(fc)
    #print(Ac)
    #print("__")

    return fc, Ac



def compute_cp_RMP_vec(xi_global, dxidt_global, posi_g, posi_o, jend, i):
    """ベクトル化で各制御点のRMPcを計算
    ・かなり早い
    """
    #alpha_o = 0
    if i == 0 or i == 1 or i == 11:  # 先頭３つ
        alpha_o = 1  # 障害物ゲイン
        beta_o = 1  # 障害物ダンピング
        gamma_o = 0.5 # 障害物オフセット
    elif 1 < i < 5 or 7 < i < 11:  # 左右側面３つずつ
        alpha_o = 1  # 障害物ゲイン
        beta_o = 0.1  # 障害物ダンピング
        gamma_o = 0.3 # 障害物オフセット
    else:  # ケツ3つ
        alpha_o = 0.0001
        beta_o = 0.0001
        gamma_o = 0.0001

    dis_obs = np.linalg.norm(posi_o - xi_global, axis = 0)
    u_all =  (posi_o - xi_global) / dis_obs
    fo_all = -alpha_o / dis_obs * (beta_o * (dxidt_global.T @ u_all) + gamma_o) * u_all
    
    
    w_all = expo(1, -1, 0, dis_obs)

    A11_all = fo_all[0:1, :] ** 2 * w_all
    A22_all = fo_all[1:2, :] ** 2 * w_all
    A12_all = fo_all[0:1, :] * fo_all[1:2, :] * w_all
    Ao_all_upper = np.insert(A11_all, np.arange(1, jend+1), A12_all, axis = 1)
    Ao_all_under = np.insert(A12_all, np.arange(1, jend+1), A22_all, axis = 1)
    Ao_all = np.concatenate([Ao_all_upper, Ao_all_under], axis = 0)

    fc_Left_term = Ao_all @ np.tile(np.eye(2, dtype = np.float32), (jend, 1))

    Af1_all = fo_all[0:1, :] * (A11_all + A22_all)
    Af2_all = fo_all[1:2, :] * (A11_all + A22_all)

    fc_Right_term = np.array([[np.sum(Af1_all)], [np.sum(Af2_all)]])

    dis_goal = np.linalg.norm(posi_g - xi_global)
    if i == 0 or i == 1 or i == 11:
        ## パラメータ状態依存
        #max_speed = 0.5
        #gain_head_max = 0.1 * 1e-2
        #gain_head_min = 0.0001
        #gain_rl_max = 0.6 * 1e-2
        #gain_rl_min = 0.0006
        #epsi_g = 1 * 10e-7

        #gain_head = sigmoid(multi = gain_head_max - gain_head_min,
        #                    coeff = 2.5,
        #                    x_offset = 2,
        #                    y_offset = gain_head_min,
        #                    x = dis_goal)
        #gain_rl = sigmoid(multi = gain_rl_max - gain_rl_min,
        #                  coeff = 2.5,
        #                  x_offset = 2,
        #                  y_offset = gain_rl_min,
        #                  x = dis_goal)
        #if i == 0:
        #    alpha_g = gain_head
        #else:
        #    alpha_g = gain_rl

        #beta_g = alpha_g / max_speed
        #fgi = alpha_g * (posi_g - xi_global) / np.linalg.norm(posi_g - xi_global + epsi_g, axis = 0) - beta_g * dxidt_global
        
        # 元のまま


        fgi = fgi_cal(xi_global, dxidt_global, posi_g, i)
        Agi = np.eye(2, dtype = np.float32)

        ## マニピュレータのやつ
        #fgi = f_attract(xi_global, dxidt_global, posi_g, alpha_g[i], beta_g[i], 1)
        #Agi = A_attract(xi_global, dxidt_global, posi_g, fgi, 1)

        fc_Left_term = fc_Left_term + Agi
        fc_Right_term  = fc_Right_term + Agi @ fgi

    fc = np.linalg.pinv(fc_Left_term) @ fc_Right_term
    Ac = fc_Left_term

    return fc, Ac



#def Jd_cal(q, x_global, x_local, posi_o, i, s):
#    """1dタスク写像のヤコビ行列"""

#    C = cos(q[2, 0])
#    S = sin(q[2, 0])
#    x = x_local[0, 0]
#    y = x_local[1, 0]
#    X = x_global[0, 0]
#    Y = x_global[1, 0]
#    ox = posi_o[0, 0]
#    oy = posi_o[1, 0]
#    return 1/s * np.array([[C * x - S * y + X - ox],
#                           [S * x + C * y + Y - oy],
#                           [(X - ox) * (-S * x - C * y) + (Y - oy) * (C * x - S * y)]], np.float32).T



def Jpsi_cal(x_global, posi_o):
    """サブタスク写像のヤコビ行列"""
    z = 1 / np.linalg.norm(x_global - posi_o) * (x_global - posi_o).T
    return z


def RMP_obs_fromGDS(x, dxdt, posi_o, i):
    """GDSからのRMPを使用"""

    s = np.linalg.norm(x - posi_o)
    Jpsi = Jpsi_cal(x, posi_o)

    dsdt_mat = Jpsi @ dxdt
    dsdt = dsdt_mat[0, 0]
    #print("Jpsi =", Jpsi)
    #print("dxdt =", dxdt.T)
    print("dsdt = ", dsdt)
    rw = 20  # 安全半径
    sigma = 1  # ？
    alpha = 1
    #print("dsdt = ", dsdt)
    w = max(rw - s, 0) ** 2 / s
    if rw - s > 0:
        dwds = -(rw ** 2 - s ** 2) / (s ** 2)
    else:
        dwds = 0

    if dsdt < 0:
        #print("1")
        u = 1 - np.exp(-(dsdt ** 2) / (2 * sigma ** 2))
        m = w * (u + 1/(2 * sigma ** 2) * dsdt ** 2 * np.exp(-(dsdt ** 2) / (2 * sigma ** 2)))
        a = dwds * (-alpha * w ** 2 - 1/2 * u * dsdt ** 2) / m
        print("w=", w)
    else:
        #print("2")
        m = 0
        a = 0

    print("a = ", a, "m = ", m)
    return np.array([[a]], np.float32), np.array([[m]], np.float32)



def RMP_obs_fromMultiLob(x, dxdt, posi_o, i):
    """マルチロボットからの障害物RMP"""
    s = np.linalg.norm(x - posi_o)
    Jpsi = Jpsi_cal(x, posi_o)

    dsdt_mat = Jpsi @ dxdt
    dsdt = dsdt_mat[0, 0]
    
    eta = 10
    alpha = 1
    epsi = 0.00001

    w = 1 / (s ** 4)
    u = epsi + min(0, dsdt) * dsdt
    pot = 1/2 * alpha * (w) ** 2

    dwdx = -4 / (s ** 5)

    if dsdt < 0:
        duddx = epsi + 2 * dsdt
    else:
        duddx = 0

    nabla_pot = alpha * w * dwdx


    G = w * u
    B = eta * G
    
    XiG = 1/2 * dsdt * w * duddx
    xiG = dwdx * (dsdt - 1/2 * (dsdt ** 2) * u)

    M = G * XiG

    if abs(M) < 1e-5:
        a = 0
        M = 0
    else:
        a = 1/M * (-nabla_pot - B * dsdt - xiG)

    #print("a=", a)
    #print("M=", M)
    return np.array([[a]], np.float32), np.array([[M]], np.float32)


def compute_cp_RMP2_nonvec(xi_global, dxidt_global, posi_g, posi_o, jend, i):
    """GDS由来のRMP
    """

    if i == 0 or i == 1 or i == 11:
        #fc_Right_term = f_attract(xi_global, dxidt_global, posi_g.T, i)
        #fc_Left_term = A_attract(xi_global, dxidt_global, posi_g.T, fc_Right_term, i)
        
        fc_Right_term = fgi_cal(xi_global, dxidt_global, posi_g.T, i)
        fc_Left_term = Agi_cal()

    else:
        fc_Right_term = np.zeros((2, 1), dtype = np.float32)
        fc_Left_term = np.zeros((2, 2), dtype = np.float32)

    for j in np.arange(0, jend, 1):
        Jpsi = Jpsi_cal(xi_global, posi_o[j:j+1, :].T)
        fo, Ao = RMP_obs_fromMultiLob(xi_global, dxidt_global, posi_o[j:j+1, :].T, i)
        
        #print(j)
        #print(Ao @ fo)
        #print(Ao)

        fc_Right_term  = fc_Right_term + Jpsi.T @ Ao @ fo
        fc_Left_term = fc_Left_term + Jpsi.T @ Ao @ Jpsi
        

    fc = np.linalg.pinv(fc_Left_term) @ fc_Right_term
    Ac = fc_Left_term

    #print("i=",i)
    #print(fc)
    #print(Ac)
    #print("__")

    return fc, Ac