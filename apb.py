import json
import os
import random
from units import whichpower
import ns3ai_apb_py_stru as py_binding
from ns3ai_utils import Experiment
import sys
import traceback
from channel_power.wolf_agent import WoLFAgent 
from channel_power.single_matrix_game import MatrixGame
from channel_power.train import MyRLEnvironment
from DQN import DeepQAgent
from env_init.weather_pred import predict
from UserActionSort import execute_cluster_operations
from env_init import elec_pred
config_file_path = '/home/ns3/project/InitConfig.json'
optimize_path = '/home/ns3/project/optimize/1.txt'
def write_opt(str):
    with open(optimize_path, 'a') as file:
        file.write(str+'\n')
        print('写入信息到1.txt:'+str)
    command = "sshpass -p 123 scp /home/ns3/project/optimize/1.txt alex@192.168.56.100:/C:/test/bin_x64/plugins/com_proj_networksimvisualization/data/optimize/1.txt"
    exit_code = os.system(command)
    # 检查命令执行结果
    if exit_code == 0:
        print("替换优化文字命令执行成功！")
    else:
        print("命令优化文字失败！")


#读取初始化设置
with open(config_file_path, 'r') as file:
    config_data = json.load(file)

print("-----------------apb.py running-------------------")



# 预先保留的天气识别和地形识别的路径
weather_path = ''
terrain_path = ''
AutomaticOptimization = config_data['AutomaticOptimization']
weather = config_data['Weather']
terrain = config_data['Terrain']
if terrain == 'upland':
    terrain = 'mountain'
if AutomaticOptimization == False:
    autoOpt = 0
################################################################################################
agent = DeepQAgent()
################################################################################################

exp = Experiment("ns3ai_apb_msg_stru", "../../../../../", py_binding,
                 handleFinish=True)
msgInterface = exp.run(show_output=True)


################################################################################################
# 导入agent，预训练1000个epoch
nb_episode = 1000
my_rl_env = MyRLEnvironment(nb_episode=nb_episode)
my_rl_env.train()
agent = my_rl_env.get_agent_instance()
# time.sleep(1)
################################################################################################
weather_map = {
    "sunny": "晴天",
    "rainy": "雨天",
    "thunderstorm": "雷暴",
    "snowy": "雪天",
    "mountain": "山地",
    "city": "城市",
    "plain": "平原",
    "forest": "森林"
}
msgInterface.PySendBegin()
msgInterface.GetPy2CppStruct().autoOpt = 1 if AutomaticOptimization else 0
msgInterface.PySendEnd()
if AutomaticOptimization:
    msgInterface.PyRecvBegin()
    time = msgInterface.GetCpp2PyStruct().time
    msgInterface.PyRecvEnd()


#优化——环境——2
if weather == 'sunny' and terrain == 'plain' and AutomaticOptimization:
    msgInterface.PySendBegin()
    msgInterface.GetPy2CppStruct().opt = 3 #速度优先
    msgInterface.PySendEnd()
    write_opt('time:{}----通信环境为{}下的{},调整传输策略为高速传输策略'.format(time, weather_map[weather], weather_map[terrain]))
elif AutomaticOptimization:
    msgInterface.PySendBegin()
    msgInterface.GetPy2CppStruct().opt = 4 #信号优先
    msgInterface.PySendEnd()
    write_opt('time:{}----通信环境为{}下的{},调整传输策略为高可靠传输策略'.format(time, weather_map[weather], weather_map[terrain]))

try_count = 0
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
#优化——电磁——5种
        #执行抗电磁
        #调用识别算法
        if action == 1 and not AutomaticOptimization:#手动
            msgInterface.PyRecvBegin()
            id = msgInterface.GetCpp2PyStruct().id
            time = msgInterface.GetCpp2PyStruct().time
            msgInterface.PyRecvEnd()
            if id == 1:
                print('id=1')
                write_opt('time:{}----通信质量优先策略添加成功'.format(time))
            if id == 2:
                print('id=2')
                write_opt('time:{}----通信速率优先策略添加成功'.format(time))
            if id == 3:
                next_channel = (current_channel+4)%13+1 #向后切换信道
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----调用单音频抗干扰通信方案：执行信道切换 {}信道---->{}信道'.format(time,current_channel,next_channel))
            if id == 4:
                msgInterface.PySendBegin()
                next_channel = (current_channel+random.randint(0,12))%13+1 #随机切换信道
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----调用多频干扰抵御通信方案：执行信道切换 {}信道---->{}信道; 执行功率提升'.format(time,current_channel,next_channel))
            if id == 5:
                write_opt('time:{}----调用电部分频带噪声干扰抑制策略策略：功率提升'.format(time))
            if id == 6:
                write_opt('time:{}----调用梳状谱及噪声调频干扰抵御方案：切换至5GHz频段'.format(time))
            if id == 7:
                RLaction = agent.act()
                next_channel = RLaction[0]
                if RLaction[1] > 20:
                    next_power = 1
                else:
                    next_power = 0
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().next_power = next_power
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                if next_power == 0:
                    write_opt('time:{}----智能优化：执行信道切换 {}信道---->{}信道; 执行功率提升'.format(time,current_channel,next_channel))
                else:
                    write_opt('time:{}----智能优化：执行信道切换 {}信道---->{}信道; 执行功率降低'.format(time,current_channel,next_channel))                    
                try_count+=1
                if try_count > 5:
                    continue
            if id == 8:
                write_opt('time:{}----应急卫星通信备用策略'.format(time))
            if id == 9:
                write_opt('time:{}----调用频谱资源优化管理'.format(time))
            if id == 10:
                write_opt('time:{}----调用能源优化管理策略'.format(time))
        if action == 3 and AutomaticOptimization:
            print('不执行自动优化')
            elec_type = elec_pred.predict("/home/ns3/project/electric/01.png")
            os.system('sshpass -p 123 scp /home/ns3/project/optimize/elec/{}.png alex@192.168.56.100:/C:/test/bin_x64/plugins/com_proj_networksimvisualization/data/identify/elect.png'.format(elec_type))
            print("识别到干扰类型:  {}".format(elec_type))
            if elec_type == "CW":
                #单音干扰，直接切换信道opt=10
                print('单音干扰优化')
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 10
                msgInterface.PySendEnd()
                msgInterface.PySendBegin()
                next_channel = (current_channel+4)%13+1 #向后切换信道
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----检测到单音干扰，调用电磁干扰优化策略：执行信道切换 {}信道---->{}信道'.format(time,current_channel,next_channel))
            elif elec_type == "PBNJ" :
                #部分频带噪声干扰--提高功率opt=11
                print('部分频带噪声干扰')
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 11
                msgInterface.PySendEnd()                
                write_opt('time:{}----检测到部分频带噪声干扰，调用电磁干扰优化策略：功率提升'.format(time, elec_type))
            elif elec_type == "NBNJ" or elec_type == "NFM" or elec_type == "CSNJ":
                #NFM噪声调频干扰--切换频段opt=12
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 12
                msgInterface.PySendEnd()    
                print('NFM噪声调频干扰--切换频段')
                write_opt('time:{}----检测到{}干扰，调用电磁干扰优化策略：切换至5GHz频段'.format(time, elec_type))
            elif elec_type == "MTJ":
                #MTJ噪声调频干扰--切换频段opt=13
                print('多音干扰优化')
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 13
                msgInterface.PySendEnd()
                msgInterface.PySendBegin()
                next_channel = (current_channel+random.randint(0,12))%13+1 #随机切换信道
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                write_opt('time:{}----检测到多音干扰，调用电磁干扰优化策略：执行信道切换 {}信道---->{}信道; 执行功率提升'.format(time,current_channel,next_channel))
            else:
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().opt = 14
                msgInterface.PySendEnd()
                RLaction = agent.act()
                next_channel = RLaction[0]
                if RLaction[1] > 20:
                    next_power = 1
                else:
                    next_power = 0
                msgInterface.PySendBegin()
                msgInterface.GetPy2CppStruct().next_power = next_power
                msgInterface.GetPy2CppStruct().next_channel = next_channel
                msgInterface.PySendEnd()
                if next_power == 0:
                    write_opt('time:{}----智能优化：执行信道切换 {}信道---->{}信道; 执行功率提升'.format(time,current_channel,next_channel))
                else:
                    write_opt('time:{}----智能优化：执行信道切换 {}信道---->{}信道; 执行功率降低'.format(time,current_channel,next_channel))                    
                try_count+=1
                if try_count > 5:
                    continue
        elif action == 4:
            try:
                execute_cluster_operations()
            except Exception as e:
                print("An error occurred while finding closest sample index:", str(e))
#优化——卫星——1种
        elif action == 5:
            continue
#优化——频谱管理——1种
        elif action == 6:
            msgInterface.PySendBegin()
            msgInterface.GetPy2CppStruct().opt = 101 #频谱分组
            msgInterface.PySendEnd()
#优化——能量管理——1种
        elif action == 7:
            continue



        

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
