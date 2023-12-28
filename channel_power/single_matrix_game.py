import numpy as np

class MatrixGame():

    def __init__(self):
        self.Cp = 1 #功率代价
        self.Cf = 5 #切换信道代价

    # def step(self, action, obs):
    #     '''
    #     action  [下一信道，下一功率]

    #     obs
    #     'current_channel':0,
    #     'current_power':1,
    #     'current_disturbed_channel':2,
    #     'current_snr':3
    #     '''
    #     K = 1 if int(obs[0]) != int(action[0]) else 0
    #     if(float(obs[3])>=15):
    #         reward = 1 - self.Cp * (int(obs[1])/24) -self.Cf * K
    #     else:
    #         reward = -1 - self.Cp * (int(obs[1])/24) -self.Cf * K

        
    #     return reward
    def step(self, action, obs):
        '''
        action  [下一信道，下一功率]

        obs
        'current_channel':current_channel,
        'current_power':current_power,
        'current_disturbed_channel':current_disturbed_channel,
        'current_snr':current_snr
        '''
        K = 1 if int(obs['current_channel']) != int(action[0]) else 0
        if(float(obs['current_snr'])>=15):
            reward = 1 - self.Cp * (int(obs['current_power'])/24) -self.Cf * K
        else:
            reward = -1 - self.Cp * (int(obs['current_power'])/24) -self.Cf * K

        
        return reward