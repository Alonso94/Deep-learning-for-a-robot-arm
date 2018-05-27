import numpy as np
import armenv_real

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, merge
from keras.optimizers import Adam

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess


# Get the environment and extract the number of actions.
env = armenv_real.ArmEnv()
np.random.seed(123)
train = 1
nb_actions = 14
action_dim = 14
state_dim = 15
# Next, we build a very simple model.
actor = Sequential()
actor.add(Flatten(input_shape=(1,) + (15,)))
actor.add(Dense(50))
actor.add(Activation('relu'))
actor.add(Dense(50))
actor.add(Activation('relu'))
actor.add(Dense(50))
actor.add(Activation('relu'))
actor.add(Dense(nb_actions))
actor.add(Activation('tanh'))
if train==0:
    actor.load_weights('real_ddpg_weights_actor.h5f')
print(actor.summary())

action_input = Input(shape=(nb_actions,), name='action_input')
observation_input = Input(shape=(1,) + (15,), name='observation_input')
flattened_observation = Flatten()(observation_input)
x = merge([action_input, flattened_observation], mode='concat')
x = Dense(100)(x)
x = Activation('relu')(x)
x = Dense(100)(x)
x = Activation('relu')(x)
x = Dense(100)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(input=[action_input, observation_input], output=x)
if train==0:
    critic.load_weights('ddpg_weights_critic.h5f')
print(critic.summary())

# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
# even the metrics!
memory = SequentialMemory(limit=100000, window_length=1)

random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)

agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
                  memory=memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
                  random_process=random_process, gamma=.99, target_model_update=1e-3)
agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

if train==1:
# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.
    agent.fit(env, nb_steps=50000, visualize=False, verbose=1, nb_max_episode_steps=200)

# After training is done, we save the final weights.
    agent.save_weights('real_ddpg_weights.h5f', overwrite=True)

# Finally, evaluate our algorithm for 5 episodes.
agent.test(env, nb_episodes=5, visualize=False, nb_max_episode_steps=200)