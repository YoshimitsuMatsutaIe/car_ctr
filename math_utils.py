"""便利関数"""

import numpy as np
from math import pi, sin, cos, tan

### 超基本の関数 ###
def rotate(theta):
    """回転行列
    入力：回転角theta
    出力：二次の回転行列
    """
    return np.array([[cos(theta), -sin(theta)],
                     [sin(theta),  cos(theta)]], dtype = np.float32)

def sigmoid(multi, coeff, x_offset, y_offset, x):
    """シグモイド関数
    y = multi / (1 + np.exp(- coeff * (x - x_offset))) + y_offset
    """
    return multi / (1 + np.exp(- coeff * (x - x_offset))) + y_offset

def expo(multi, coeff, x_offset, x):
    """指数関数
    y = multi * np.exp(coeff * (x - x_offset))
    """
    return multi * np.exp(coeff * (x - x_offset))

def linear_func(a, b, x):
    """一次関数（傾き負，非負値を返す）"""
    return max(a * x + b, 0)


### 基本の関数 ###
def beta_cal(v_command):
    """βを計算"""

    # 元論文（アッカーマン）
    z = np.arctan(tan(v_command[1, 0]) / 2)

    ## ドリフト有りモデル
    #lr = 0.2
    #L = 0.4
    #z = np.arctan(lr / L * tan(v_command[1, 0]))

    return z

def xi_global_cal(xi_local, q):
    """グローバル位置を計算"""
    return rotate(q[2, 0]) @ xi_local + q[0:2, :]

def Jphii_cal(L, W, q, xi_local):
    """タスク写像のヤコビ行列"""
    return np.array([[1, 0, -sin(q[2, 0]) * xi_local[0, 0] - cos(q[2, 0]) * xi_local[1, 0]],
                     [0, 1,  cos(q[2, 0]) * xi_local[0, 0] - sin(q[2, 0]) * xi_local[1, 0]]], dtype = np.float32)
    #return np.array([[1, 0, -xi_local[1, 0]],
    #                 [0, 1,  xi_local[0, 0]]], dtype = np.float32)

def dJphiidt_cal(q, dqdt, xi_local):
    """タスク写像のヤコビ行列の時間微分
    ・RMPflowで使用
    ・要らない？
    """
    return dqdt[2, 0] * np.array([[0, 0, -cos(q[2, 0]) * xi_local[0, 0] + sin(q[2, 0]) * xi_local[1, 0]],
                                  [0, 0, -sin(q[2, 0]) * xi_local[0, 0] - cos(q[2, 0]) * xi_local[1, 0]]], dtype = np.float32)


def transform_jacobi(L, q, v_command, beta):
    """加速度指令をステアリング角速度，前方速度に変化するヤコビ？行列と追加項を計算"""

    ## 元論文を修正したやつ
    #J = np.array([[cos(q[2, 0]), 0],
    #              [sin(q[2, 0]), 0],
    #              [sin(2 * beta_cal(v_command)) / L, \
    #                  4 * v_command[0, 0] * cos(2 * beta) / (3 * (cos(v_command[1, 0]) ** 2) + 1) / L]], dtype = np.float32)  
    #T = np.array([[-v_command[0, 0] ** 2 / L * sin(q[2, 0]) * sin(2 * beta)],
    #              [ v_command[0, 0] ** 2 / L * cos(q[2, 0]) * sin(2 * beta)],
    #              [ 0]], dtype = np.float32)  # 修正版

    # 元論文（誤植？）
    J = np.array([[cos(q[2, 0]), 0],
                  [sin(q[2, 0]), 0],
                  [sin(2 * beta) / L, \
                      4 * v_command[0, 0] * cos(2 * beta) / (3 * (cos(v_command[1, 0]) ** 2) + 1)]], dtype = np.float32)
    #T = np.array([[0],
    #              [v_command[0, 0] ** 2 / L * cos(q[2, 0]) * sin(2 * beta)],
    #              [0]], dtype = np.float32)

    ## ドリフト無しモデル
    #J = np.array([[cos(q[2, 0])            , 0],
    #              [sin(q[2, 0])            , 0],
    #              [tan(v_command[1, 0]) / L, v_command[0, 0] / (L * cos(v_command[1, 0]) ** 2)]], dtype = np.float32)
    #T = np.array([[-v_command[0, 0] ** 2 / L * tan(v_command[1, 0]) * sin(q[2, 0])],
    #              [ v_command[0, 0] ** 2 / L * tan(v_command[1, 0]) * cos(q[2, 0])],
    #              [ 0]], dtype = np.float32)

    ##github？．よくわからない
    #J = np.array([[1, 0],
    #              [0, 0],
    #              [sin(2 * beta_cal(v_command)) / L, \
    #                  4 * v_command[0, 0] * cos(2 * beta) / (3 * (cos(v_command[1, 0]) ** 2) + 1)]], dtype = np.float32)
    #T = np.array([[0],
    #              [v_command[0, 0] ** 2 / L * cos(q[2, 0]) * sin(2 * beta)],
    #              [0]], dtype = np.float32)

    ## turtlebot
    #J = np.array([[cos(q[2, 0]), 0],
    #              [sin(q[2, 0]), 0],
    #              [           0, 1]], dtype = np.float32)
    #T = np.array([[-v_command[0, 0] * sin(q[2, 0]) * v_command[1, 0]],
    #              [ v_command[0, 0] * cos(q[2, 0]) * v_command[1, 0]],
    #              [ 0]], dtype = np.float32)

    ## ドリフト有りモデル
    #lr = 0.2
    #theta = q[2, 0]
    #v, xi = v_command[0, 0], v_command[1, 0]
    #J = np.array([[cos(theta + beta), -v * (lr / L) * ((cos(beta) / cos(xi)) ** 2) * sin(theta + beta)],
    #              [sin(theta + beta),  v * (lr / L) * ((cos(beta) / cos(xi)) ** 2) * cos(theta + beta)],
    #              [sin(beta) / lr,     v / L * (cos(beta) ** 2) / (cos(xi) ** 2)]], dtype = np.float32)
    #T = np.array([[-v ** 2 / lr * sin(beta) * sin(theta + beta)],
    #              [ v ** 2 / lr * sin(beta) * cos(theta + beta)],
    #              [ 0]], dtype = np.float32)

    ## T無し
    T = np.zeros((3, 1), dtype = np.float32)

    return J, T


def transform_velocity(L, q, v_command, beta):
    """速度指令を配置空間速度に変換（元論文）"""

    # 元論文（アッカーマン）
    z = np.array([[v_command[0, 0] * cos(q[2, 0])],
                  [v_command[0, 0] * sin(q[2, 0])],
                  [v_command[0, 0] / L * sin(2 * beta)]], dtype = np.float32)

    ## ドリフト無しモデル
    #z = np.array([[v_command[0, 0] * cos(q[2, 0])],
    #              [v_command[0, 0] * sin(q[2, 0])],
    #              [2 * v_command[0, 0] / L * tan(v_command[1, 0])]], dtype = np.float32)
                  
    ## turtlebot
    #z = np.array([[v_command[0, 0] * cos(q[2, 0])],
    #              [v_command[0, 0] * sin(q[2, 0])],
    #              [v_command[1, 0]]], dtype = np.float32)

    ## ドリフト有りモデル
    #lr = 0.2
    #z = np.array([[v_command[0, 0] * cos(q[2, 0] + beta)],
    #              [v_command[0, 0] * sin(q[2, 0] + beta)],
    #              [v_command[0, 0] / lr * sin(beta)]], dtype = np.float32)

    return z


     


