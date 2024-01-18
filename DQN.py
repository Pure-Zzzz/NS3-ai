import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from itertools import product
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(3, 20),
            nn.ReLU(),
            nn.Linear(20, 20),
            nn.ReLU(),
            nn.Linear(20, 4),
        )

    def forward(self, x):
        return self.layers(x)

class DQN:
    def __init__(self):
        self.eval_net = Net()
        self.target_net = Net()
        self.learn_step = 0
        self.batchsize = 32
        self.observer_shape = 3  # 修改观测空间的维度
        self.action_space = 4  # 修改动作空间的维度
        self.target_replace = 100
        self.memory_counter = 0
        self.memory_capacity = 2000
        self.memory = np.zeros((2000, 2*self.observer_shape + 2))  # s, a, r, s'，调整维度
        self.optimizer = optim.Adam(
            self.eval_net.parameters(), lr=0.0001)
        self.loss_func = nn.MSELoss()
        

    def choose_action(self, x):
        x = torch.Tensor(x)
        if np.random.uniform() > 0.99 ** self.memory_counter:  # choose best
            action = self.eval_net(x)
            action = torch.argmax(action, 0).numpy()
        else:  # explore
            action = np.random.randint(0, self.action_space)
        return action

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        self.memory[index, :] = np.hstack((s, [a, r], s_))
        self.memory_counter += 1

    def learn(self):
        self.learn_step += 1
        if self.learn_step % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        sample_list = np.random.choice(self.memory_capacity, self.batchsize)
        sample = self.memory[sample_list, :]
        s = torch.Tensor(sample[:, :self.observer_shape])
        a = torch.LongTensor(
            sample[:, self.observer_shape:self.observer_shape + 1])
        r = torch.Tensor(
            sample[:, self.observer_shape + 1:self.observer_shape + 2])
        s_ = torch.Tensor(sample[:, self.observer_shape + 2:])
        q_eval = self.eval_net(s).gather(1, a)
        q_next = self.target_net(s_).detach()
        q_target = r + 0.8 * q_next.max(1, True)[0]

        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
'''
--action
//rxgain（加，减）
//txgain（加，减）
mcs（加，减）
power（加，减）
action=[rxgain,txgain,mcs,power]____(0,1)
--obs
snr
delay
Tput
'''
class DeepQAgent:
    def __init__(self):
        self.dqn = DQN()
        self.new_rxgain = None
        self.new_txgain = None
        self.new_mcs = None
        self.new_power = None
        self.s = None #last state
        self.a = None
        self.r = None
        self.s_ = None  # next state
        self.act =  list(product([0, 1], repeat=2))

    def get_action(self, obs):
        snr = obs[0]
        delay = obs[1]
        Tput = obs[2]
        weight_snr = 0.7  # SNR 权重
        weight_delay = 0.1  # Delay 权重
        weight_tput = 0.2  # Tput 权重
        # update DQN
        self.s = self.s_
        self.s_ = obs
        if self.s:  # not first time
            self.r =  (
            weight_snr * snr +
            weight_delay * (1 / (1 + delay)) +  # 考虑 delay 越小越好
            weight_tput * Tput  # 考虑 Tput 越接近目标越好
        )# calculate your reward based on the new state and action
            self.dqn.store_transition(self.s, self.a, self.r, self.s_)
            if self.dqn.memory_counter > self.dqn.memory_capacity:
                self.dqn.learn()

        # choose action
        self.a = self.dqn.choose_action(self.s_)
        
        return self.act[self.a]
