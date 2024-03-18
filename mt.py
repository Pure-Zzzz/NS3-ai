from channel_power.train import MyRLEnvironment
nb_episode = 1000
my_rl_env = MyRLEnvironment(nb_episode=nb_episode)
my_rl_env.train()
agent = my_rl_env.get_agent_instance()
print(agent.act()[0])