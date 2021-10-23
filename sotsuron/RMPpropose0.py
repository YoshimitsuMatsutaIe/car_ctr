"""統合プログラムファイル
・ここで関数を読み込んで実行させる
・ローカル関数importしまくると遅くなるかも？
"""

import math

from localminimum import *

from main_func import * 
from main_func_2 import *

from main_exp import *
from car_dynamics import *
from make_figs import *

#mainsimu()
#mainsimu_2()

#minical()

#carsimu_2(q = np.array([[0, 0, -pi/4]]).T, 
#          dqdt = np.array([[0, 0, 0]]).T, 
#          v_command = np.array([[0.1, math.pi / 100]]).T, 
#          beta_slip = np.array([[0]]))

#main_exp_simu()


#make_youshi_fig(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__従来手法ためし静的2021-02-15--06-19-55__State_history_temp.csv',
#                r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\timeover__提案手法ためし静的2021-02-15--06-24-43__State_history_temp.csv',
#                r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\timeover__提案手法ためし静的2021-02-15--06-24-43__obs_history_temp.csv')

#make_youshi_fig(r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\collision__従来手法ためし動的2021-02-15--06-55-17__State_history_temp.csv',
#                r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__提案手法ためし動的2021-02-15--06-56-55__State_history_temp.csv',
#                r'C:\Users\Elis\Documents\制御2020\リーマン制御2020\Exp_Data_scgool\卒論用\succes__提案手法ためし動的2021-02-15--06-56-55__obs_history_temp.csv')


#make_soturon_fig()

#make_soturon_fig_iroiro()

make_soturon_gif()