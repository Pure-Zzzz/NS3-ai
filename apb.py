# Copyright (c) 2019-2023 Huazhong University of Science and Technology
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Author: Pengyu Liu <eic_lpy@hust.edu.cn>
#         Xiaojun Guo <guoxj@hust.edu.cn>
#         Hao Yin <haoyin@uw.edu>
#         Muyuan Shen <muyuan_shen@hust.edu.cn>


import ns3ai_apb_py_stru as py_binding
from ns3ai_utils import Experiment
import sys
import traceback
import numpy as np
import matplotlib.pyplot as plt
from channel_power.wolf_agent import WoLFAgent 
from channel_power.single_matrix_game import MatrixGame
import pandas as pd
import numpy as np
import random
import time
from channel_power.train import MyRLEnvironment
#开启初始化
# from env_init import env_init
# env_init.run()

exp = Experiment("ns3ai_apb_msg_stru", "../../../../../", py_binding,
                 handleFinish=True)
msgInterface = exp.run(show_output=True)


# 导入agent，预训练1000个epoch
# nb_episode = 1000
# my_rl_env = MyRLEnvironment(nb_episode=nb_episode)
# my_rl_env.train()
# agent = my_rl_env.get_agent_instance()
# time.sleep(1)



try:
    while True:
        # receive from C++ side
        print('python开始接收')
        msgInterface.PyRecvBegin()
        # cpp_act = msgInterface.GetCpp2PyStruct().cpp_action
        if msgInterface.PyGetFinished():
            break
        id = msgInterface.GetCpp2PyStruct().id
        print("id: {}".format(id))
        print('python结束接受')
        msgInterface.PyRecvEnd()

        # action = agent.act()

        # send to C++ side
        msgInterface.PySendBegin()
        msgInterface.GetPy2CppStruct().next_channel = 5
        msgInterface.GetPy2CppStruct().next_power = 110
        msgInterface.PySendEnd()

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    print("Traceback:")
    traceback.print_tb(exc_traceback)
    exit(1)



else:
    pass

finally:
    print("Finally exiting...")
    del exp
