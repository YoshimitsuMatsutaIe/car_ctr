""""車体形状関係"""

import numpy as np

def car_model1():
    """現実的な車？
    ・長谷川さんのモデル
    """

    lf = 0.9560  # 正面から重心までの距離[m]
    lr = 1.434  # 後ろから重心までの距離[m]
    L = lf + lr  # 全長
    m = 1070  # 車両質量[kg]
    I = 2727 # 車両の重心回りの慣性モーメント[sec^2/rad]
    Kf = 1250  # コーナリングパワー係数[N/rad]．仮定：スリップ角が微笑
    Kr = 1050  # コーナリングパワー係数[N/rad]．仮定：スリップ角が微笑

    car_param = [lf, lr, L, m, I, Kf, Kr]

    return car_param
