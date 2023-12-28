import numpy as np
import matplotlib.pyplot as plt
from wolf_agent import WoLFAgent 
from single_matrix_game import MatrixGame
import pandas as pd
import numpy as np
import random

def load_data(file_path):
    data = []
    channel = 1
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            channel = line.split()[0]
            snr = line.split()[-1]
            if snr == 'N/A':
                snr = 0
            observation = {
                'channel': channel,
                'snr': snr
                # 可根据需要添加其他观测值
            }
            data.append(observation)
    return data

# 示例使用
file16 = "data/power16.txt"
file20 = "data/power20.txt"
file24 = "data/power24.txt"
power16 = load_data(file16)
power20 = load_data(file20)
power24 = load_data(file24)


if __name__ == '__main__':
    nb_episode = 100
    # 通信信道和发射功率的选择列表
    channels = np.arange(1, 14)  # 1到13共13个通信信道
    powers = np.array([16, 20, 24])  # 三个发射功率选择
    # 生成所有可能的动作组合
    actions = np.array(np.meshgrid(channels, powers)).T.reshape(-1, 2)
    # 构建环境感知
    power16 = load_data(file16)
    power20 = load_data(file20)
    power24 = load_data(file24)


    agent1 = WoLFAgent(alpha=0.1, actions=actions, high_delta=0.0004, low_delta=0.0002) 
    current_channel = random.randint(1,13)
    current_disturbed_channel = random.randint(1,13)

    current_power = random.choice([16,20,24])
    if current_power == 16:
        current_snr = power16[current_channel+(current_disturbed_channel-1)*13-1]['snr']
    elif current_power == 20:
        current_snr = power20[current_channel+(current_disturbed_channel-1)*13-1]['snr']
    elif current_power == 24:
        current_snr = power24[current_channel+(current_disturbed_channel-1)*13-1]['snr']
    obs = {
        'current_channel':current_channel,
        'current_power':current_power,
        'current_disturbed_channel':current_disturbed_channel,
        'current_snr':current_snr
           }
    
    game = MatrixGame()
    # 训练代理，执行一定数量的训练步骤
    for episode in range(nb_episode):
        action = agent1.act()
        reward = game.step(action, obs)
        agent1.observe(reward)

    # 循环执行step，直到找到满足条件的动作
    while True:
        action = agent1.act()
        reward = game.step(action, obs)
        agent1.observe(reward)

        # 获取下一步SNR
        if action[1] == 16:
            next_snr = power16[action[0] + (current_disturbed_channel - 1) * 13 - 1]['snr']
        elif action[1] == 20:
            next_snr = power20[action[0] + (current_disturbed_channel - 1) * 13 - 1]['snr']
        elif action[1] == 24:
            next_snr = power24[action[0] + (current_disturbed_channel - 1) * 13 - 1]['snr']

        print("下一步SNR: {}".format(next_snr))

        # 检查是否满足条件
        if float(next_snr) > 15:
            break  # 如果满足条件，退出循环

    # 执行最佳动作
    best_action = np.argmax(agent1.q_values)
    action = actions[best_action]
    print("当前信道:{}  当前功率:{} 当前受干扰的信道:{} 当前的SNR:{}    ".format(current_channel, current_power, current_disturbed_channel, current_snr))
    print("找到满足条件的动作：{}".format(action))