import json
import os
import random
import shutil
import units
from units import whichpower
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
from channel_power.train import MyRLEnvironment
from DQN import DeepQAgent
from env_init.weather_pred import predict
import subprocess
from UserActionSort import execute_cluster_operations
from env_init import elec_pred
def delete_files_in_folder(path):
    try:
        print("python端：删除{}路径下文件".format(path))
        subprocess.run('rm ' + path, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

def write_opt(str):
    with open(optimize_path, 'a') as file:
        file.write(str+'\n')
        print('写入信息到opt.txt:'+str)

config_file_path = '/home/ns3/project/InitConfig.json'
with open(config_file_path, 'r') as file:
    config_data = json.load(file)

print("-----------------apb.py running-------------------")

optimize_path = '/home/ns3/project/optimize/opt.txt'
weather_path = ''
terrain_path = ''

weather = 'sunny'
terrain = config_data['Terrain']
if terrain == 'upland':
    terrain = 'mountain'
################################################################################################
agent = DeepQAgent()
exp = Experiment("ns3ai_apb_msg_stru", "../../../../../", py_binding,
                 handleFinish=True)
msgInterface = exp.run(show_output=True)
################################################################################################
# 删除配置文件
os.remove(config_file_path)
print(f"{config_file_path} 文件已删除")
################################################################################################
# 导入agent，预训练1000个epoch
# nb_episode = 1000
# my_rl_env = MyRLEnvironment(nb_episode=nb_episode)
# my_rl_env.train()
# agent = my_rl_env.get_agent_instance()
# time.sleep(1)
################################################################################################
if weather == 'sunny' and terrain == 'plain':
    msgInterface.PySendBegin()
    msgInterface.GetPy2CppStruct().opt = 3 #速度优先
    msgInterface.PySendEnd()
else:
    msgInterface.PySendBegin()
    msgInterface.GetPy2CppStruct().opt = 4 #信号优先
    msgInterface.PySendEnd()


try:
    while True:
        # receive from C++ side
        print('python开始接收')
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        action = msgInterface.GetCpp2PyStruct().action
        current_channel = msgInterface.GetCpp2PyStruct().current_channel
        time = msgInterface.GetCpp2PyStruct().time
        msgInterface.PyRecvEnd()

        print("action: {}".format(action))
        print("current_channel: {}".format(current_channel))

        if action==3:
            #执行抗电磁
            #调用识别算法
            # elc_class = 'SNM'
            elec_type = elec_pred.predict("/home/ns3/project/electric/01.png")
            print("识别到干扰类型:  {}".format(elec_type))
            if elec_type == "CW":
                #单音干扰，直接切换信道
                print('单音干扰优化')
                delete_files_in_folder("/home/ns3/project/electric/*")
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 0
                msgInterface.PySendEnd()
                msgInterface.PySendBegin()
                next_channel = (current_channel+4)%13+1 #随机切换信道
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----检测到单音干扰，调用电磁干扰优化策略：执行信道切换 {}信道---->{}信道'.format(time,current_channel,next_channel))
            elif elec_type == "NBNJ" or elec_type == "NFM" or elec_type == "PBNJ" :
                #部分频带噪声干扰--提高功率
                print('部分频带噪声干扰')
                delete_files_in_folder("/home/ns3/project/electric/*")
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 1 
                msgInterface.PySendEnd()                
                write_opt('time:{}----检测到部分频带噪声干扰，调用电磁干扰优化策略：功率提升')
            elif elec_type == "MTJ":
                print('多音干扰优化')
                delete_files_in_folder("/home/ns3/project/electric/*")
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 2
                msgInterface.PySendEnd()
                msgInterface.PySendBegin()
                next_channel = (current_channel+random.randint(2,12))%13+1 #随机切换信道
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----检测到多音干扰，调用电磁干扰优化策略：执行信道切换 {}信道---->{}信道'.format(time,current_channel,next_channel))
        elif action==4:
            try:
                execute_cluster_operations()
            except Exception as e:
                print("An error occurred while finding closest sample index:", str(e))
            # msgInterface.PySendBegin()
            # msgInterface.GetPy2CppStruct().opt = 2
            # msgInterface.PySendEnd()
            #执行活跃度调整

        
        
        {# snr = msgInterface.GetCpp2PyStruct().snr
        # delay = msgInterface.GetCpp2PyStruct().delay
        # tput = msgInterface.GetCpp2PyStruct().tput
        # current_power = msgInterface.GetCpp2PyStruct().current_power
        # nodetype = msgInterface.GetCpp2PyStruct().nodetype
        # mcs = msgInterface.GetCpp2PyStruct().mcs
        # msgInterface.PyRecvEnd()
        # obs = [snr, delay, tput]
        # power_ch = whichpower(nodetype)
        # index = power_ch.index(current_power)
        # act = agent.get_action(obs)
        # if act[0]==1 and index+1<len(power_ch):
        #     index+=1
        # elif act[0]==0 and index!=0:
        #     index-=1
        # else:
        #     index
        # if act[1]==1 and mcs!=7 and mcs!=-1:
        #     mcs+=1
        # elif act[1]==0 and mcs!=0 and mcs!=-1:
        #     mcs-=1
        # else:
        #     mcs
        # send to C++ side
        # msgInterface.PySendBegin()
        # msgInterface.GetPy2CppStruct().next_power = power_ch[index]
        # msgInterface.GetPy2CppStruct().next_mcs = mcs
        # msgInterface.PySendEnd()}
        }

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
