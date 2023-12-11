# Copyright (c) 2023 Huazhong University of Science and Technology
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
# Author: Muyuan Shen <muyuan_shen@hust.edu.cn>


import ns3ai_gym_env
import gymnasium as gym
import sys
import traceback
import warnings
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import random
#屏蔽警告
warnings.filterwarnings("ignore", category=UserWarning)

# APB_SIZE = 3

MAX_MEMORY_SIZE = 100
class DQNAgent(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQNAgent, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.memory = []  # 存储经验的记忆
        self.gamma = 0.95  # 折扣因子
        self.epsilon = 1.0  # 探索-利用权衡参数
        self.epsilon_decay = 0.995  # 探索-利用权衡参数的衰减率
        self.epsilon_min = 0.01  # 最小探索概率
        self.learning_rate = 0.001  # 学习率

        # 创建深度Q网络
        self.fc1 = nn.Linear(self.state_size, 32)
        self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(32, self.action_size)

        self.optimizer = optim.Adam(self.parameters(), lr=self.learning_rate)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > MAX_MEMORY_SIZE:
            self.memory.pop(0)  # 移除最早的经验

    def act(self, state):
        # epsilon贪心策略，以一定概率探索，以一定概率根据Q值选择动作
        if np.random.rand() <= self.epsilon:
            return [np.random.choice(self.action_size)]
        q_values = self.model(torch.Tensor(state))
        return [np.argmax(q_values.detach().numpy())]

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        states, targets = [], []

        for state, action, reward, next_state, done in minibatch:
            state = torch.tensor(state, dtype=torch.float32).view(1, -1)
            next_state = torch.tensor(next_state, dtype=torch.float32).view(1, -1)

            target = reward
            if not done:
                target = reward + self.gamma * torch.max(self.forward(next_state))
            target_f = self.forward(state)
            target_f[0, action] = target

            states.append(state)
            targets.append(target_f)

        states = torch.cat(states)
        targets = torch.cat(targets)

        loss = F.mse_loss(targets, self.forward(states))
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

agent = DQNAgent(3, 2)

print('---------------------------PYTHON 初始化env----------------------------')

env = gym.make("ns3ai_gym_env/Ns3-v0", targetName="scratch/testnewns", ns3Path="/home/ns3/ns-allinone-3.40/ns-3.40/")
ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)
num_episodes = 3
batch_size = 5

for episode in range(num_episodes):
    # state = np.reshape(env.reset(), [1, state_size])
    state, info = env.reset()
    total_reward = 0

    for t in range(5):  # 限制每个episode的步数
        action = agent.act(state)
        next_state, reward, done, _, info = env.step(action)
        # next_state = np.reshape(next_state, [1, state_size])
        print('------------------------------action------------------------------')
        print(action)
        print('------------------------------state-------------------------------')
        print(state)
        print('------------------------------reward------------------------------')
        print(reward)
        print('----------------------------next state------------------------------')
        print(next_state)
        print('-------------------------------------------------------------------')

        total_reward += reward

        agent.remember(state, action, reward, next_state, done)
        state = next_state

        if done:
            break

    if len(agent.memory) > batch_size:
        agent.replay(batch_size)

    print(f"Episode: {episode + 1}, Total Reward: {total_reward}")
print("Finally exiting...")
env.close()
