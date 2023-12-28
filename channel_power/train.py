import numpy as np
from channel_power.wolf_agent import WoLFAgent
from channel_power.single_matrix_game import MatrixGame
import random
file16 = "/home/ns3/ns-allinone-3.40/ns-3.40/scratch/channel_power/data/power16.txt"
file20 = "/home/ns3/ns-allinone-3.40/ns-3.40/scratch/channel_power/data/power20.txt"
file24 = "/home/ns3/ns-allinone-3.40/ns-3.40/scratch/channel_power/data/power24.txt"
class MyRLEnvironment:

    def __init__(self, nb_episode=1000):
        self.nb_episode = nb_episode
        self.channels = np.arange(1, 14)
        self.powers = np.array([16, 20, 24])
        self.actions = np.array(np.meshgrid(self.channels, self.powers)).T.reshape(-1, 2)
        self.agent1 = WoLFAgent(alpha=0.1, actions=self.actions, high_delta=0.0004, low_delta=0.0002)
        self.game = MatrixGame()


    def load_data(self, file_path):
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
    def get_agent_instance(self):
        actions = self.generate_actions()  # 生成所有可能的动作组合
        agent_instance = WoLFAgent(alpha=0.1, actions=actions, high_delta=0.0004, low_delta=0.0002)
        # 其他初始化逻辑
        return agent_instance

    def generate_actions(self):
        # 通信信道和发射功率的选择列表
        channels = list(range(1, 14))  # 1到13共13个通信信道
        powers = [16, 20, 24]  # 三个发射功率选择
        # 生成所有可能的动作组合
        actions = [(channel, power) for channel in channels for power in powers]
        return actions

    def train(self):
        agent1 = self.get_agent_instance()
        for episode in range(self.nb_episode):
            current_channel = random.randint(1, 13)
            current_disturbed_channel = random.randint(1, 13)
            current_power = random.choice([16, 20, 24])

            if current_power == 16:
                current_snr = self.load_data(file16)[current_channel + (current_disturbed_channel - 1) * 13 - 1]['snr']
            elif current_power == 20:
                current_snr = self.load_data(file20)[current_channel + (current_disturbed_channel - 1) * 13 - 1]['snr']
            elif current_power == 24:
                current_snr = self.load_data(file24)[current_channel + (current_disturbed_channel - 1) * 13 - 1]['snr']

            obs = {
                'current_channel': current_channel,
                'current_power': current_power,
                'current_disturbed_channel': current_disturbed_channel,
                'current_snr': current_snr
            }

            game = MatrixGame()
            action = agent1.act()
            reward = game.step(action, obs)
            agent1.observe(reward)



