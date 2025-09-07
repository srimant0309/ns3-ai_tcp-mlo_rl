import os
import torch
import numpy as np
import argparse
import ns3ai_gym_env
import gymnasium as gym
import torch.nn as nn
import torch.optim as optim
import sys

#sys.stdout = open('output/python_output_10_DL_BE.txt', 'a')

# Action space values (discretized probabilities)
ACTION_SPACE_SIZE = 11
DISCRETE_ACTIONS = np.array([0, 0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95])

class Net(nn.Module):
    def __init__(self, state_dim):
        super(Net, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 3 * ACTION_SPACE_SIZE)  # 3 probability values, each with 11 discrete choice `s
        )

    def forward(self, x):
        return self.layers(x)

class DQN:
    def __init__(self, state_dim, lr=0.0001, gamma=0.9, memory_capacity=5):
        self.state_dim = state_dim
        self.gamma = gamma
        self.memory_capacity = memory_capacity
        self.memory_counter = 0
        self.batch_size = 32
        self.target_replace = 10

        # Initialize networks with correct output shape
        self.eval_net = Net(state_dim)
        self.target_net = Net(state_dim)
        
        self.optimizer = optim.Adam(self.eval_net.parameters(), lr=lr)
        self.loss_func = nn.MSELoss()
        
        # Memory buffer for replay
        self.memory = np.zeros((memory_capacity, 2 * state_dim + 4))  # 3 actions + 1 reward

    def choose_action(self, state):
        state = torch.Tensor(state)
        with torch.no_grad():
            action_logits = self.eval_net(state).numpy()
        action_logits = action_logits.reshape(3, ACTION_SPACE_SIZE)
        action_indices = np.argmax(action_logits, axis=1)
        action_values = DISCRETE_ACTIONS[action_indices]
        return action_values.tolist()  

    def store_transition(self, s, a, r, s_):
        index = self.memory_counter % self.memory_capacity
        self.memory[index, :] = np.hstack((s, a, [r], s_))
        self.memory_counter += 1

    def learn(self):
        if self.memory_counter < self.batch_size:
            return
        if self.memory_counter % self.target_replace == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())

        sample_indices = np.random.choice(self.memory_capacity, self.batch_size)
        batch = self.memory[sample_indices, :]
        s = torch.Tensor(batch[:, :self.state_dim])
        a = torch.Tensor(batch[:, self.state_dim:self.state_dim + 3])
        r = torch.Tensor(batch[:, self.state_dim + 3:self.state_dim + 4])
        s_ = torch.Tensor(batch[:, self.state_dim + 4:])

        q_eval = self.eval_net(s)
        q_target = r + self.gamma * self.target_net(s_).detach().max(1, True)[0]

        loss = self.loss_func(q_eval, q_target)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

class RLAgent:
    def __init__(self, state_dim):
        self.dqn = DQN(state_dim)
        self.current_state = None
        self.previous_state = None
        self.previous_action = None
        self.reward = None

    def get_action(self, observation, reward, done):
        self.previous_state = self.current_state
        self.current_state = np.array(observation).flatten()

        if self.previous_state is not None:
            self.dqn.store_transition(self.previous_state, self.previous_action, reward, self.current_state)
            if self.dqn.memory_counter > self.dqn.memory_capacity:
                self.dqn.learn()

        action = self.dqn.choose_action(self.current_state)
        self.previous_action = action
        return action
    
def read_observation(obs, num_stations):
    obs = obs.tolist()
    state_data = []
    for i in range(num_stations):
        base_idx = i * 6  
        state_data.append(obs[base_idx:base_idx + 6]) 
    return state_data

parser = argparse.ArgumentParser()
parser.add_argument('--traffic', type=str, default='traffic')
parser.add_argument('--num_stations', type=int, default=11)  
args = parser.parse_args()

num_stations = args.num_stations  

ns3Settings = {
    'trafficFile': args.traffic,
    'frequency': 5,
    'frequency2': 6,
    'frequency3': 2.4,
    'nEhtStations': 10,
    'radius': 20,
    'channelWidth': 40,
    'channelWidth2': 40,
    'channelWidth3': 40,
    'guardInterval': 800,
    'heRate': 11,
    'baBufferSize': 1024
}

env = gym.make("ns3ai_gym_env/Ns3-v0", targetName="ns3ai_mlo_gym", ns3Path="/home/srimant_ubuntu/research/ns-3_latest/ns-3-dev-mlo-rand-block-sched-rebased/ns-3-dev-mlo-rand-block-sched-rebased", ns3Settings=ns3Settings)

dim_state = 6  
dim_action = 3

#agents = [RLAgent(dim_state) for _ in range(num_stations)]
#if a model exists in output folder, load it otherwise create a new model
agents = []
for i in range(num_stations):
    if os.path.exists(f'/home/srimant_ubuntu/research/ns-3_latest/ns-3-dev-mlo-rand-block-sched-rebased/ns-3-dev-mlo-rand-block-sched-rebased/contrib/ai/examples/mlo/output/agent_{i}.pt'):
        agent = RLAgent(dim_state)
        agent.dqn.eval_net.load_state_dict(torch.load(f'/home/srimant_ubuntu/research/ns-3_latest/ns-3-dev-mlo-rand-block-sched-rebased/ns-3-dev-mlo-rand-block-sched-rebased/contrib/ai/examples/mlo/output/agent_{i}.pt'))
        agents.append(agent)
    else:
        agents.append(RLAgent(dim_state))

obs, info = env.reset()
obs = read_observation(obs, num_stations)
print(f"No. of stations(including AP): {num_stations}")
print(f"Initial state: {obs}")
reward = [0.0] * num_stations  
done = False
t = 0

#output_file = open('/home/srimant_ubuntu/research/ns-3_latest/ns-3-dev-mlo-rand-block-sched-rebased/ns-3-dev-mlo-rand-block-sched-rebased/contrib/ai/examples/mlo/output/rl_output_10_DL_BE.txt', 'a')

while not done:
    actions = []
    for i in range(num_stations):
        action = agents[i].get_action(obs[i], reward[i], done)
        actions.append(action)
    print(f"Actions: {actions}")
    for i in range(num_stations):
        if(i==5):
            print(f"Step: {t}, Station {i}: State: {obs[i]}, Action: {actions[i]}, Reward: {reward[i]}")
            #write to output/rl_output.txt
            #output_file.write(f"Step: {t}, Station {i}: State: {obs[i]}, Action: {actions[i]}, Reward: {reward[i]}\n")
    #output_file.write("\n")
    actions = np.array(actions).flatten()
    print(actions.shape)
    obs, reward, done, _, info = env.step(actions)
    info = info['info'].split()
    print(info)
    #reward = [float(i) for i in info]
    reward = []
    for i in info:
        try:
            x = float(i)
            reward.append(x)
        except:
            y=reward[-1]
            reward.append(y)
            reward.append(y)
    print(reward)
    obs = read_observation(obs, num_stations)
    
    print(f"Step: {t}")
    t += 1

#agents list contains the agent model for each station, save each of them
'''for i in range(num_stations):
    torch.save(agents[i].dqn.eval_net.state_dict(), f'/home/srimant_ubuntu/research/ns-3_latest/ns-3-dev-mlo-rand-block-sched-rebased/ns-3-dev-mlo-rand-block-sched-rebased/contrib/ai/examples/mlo/output/agent_{i}.pt')'''

#output_file.close()
env.close()