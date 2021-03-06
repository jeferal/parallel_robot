import gym

from time import sleep
from pr_agents import DDPGAgent

import numpy as np
import matplotlib.pyplot as plt

import tensorflow as tf


def plot_learning_curve(x, scores, figure_file):
    running_avg = np.zeros(len(scores))
    for i in range(len(running_avg)):
        running_avg[i] = np.mean(scores[max(0, i-100):(i+1)])
    plt.plot(x, running_avg)
    plt.title('Running average of previous 100 scores')
    plt.savefig(figure_file)


def main(args=None):
    env = gym.make('gym_parallel_robot:ParallelRobot-v0')

    agent = DDPGAgent(input_dims=env.observation_space.shape, env=env,
            n_actions=env.action_space.shape[0])
    
    n_games = 1000

    figure_file = 'plots/pr_avg_score.png'

    best_score = env.reward_range[0]
    score_history = []
    load_checkpoint = False

    if load_checkpoint:
        n_steps = 0
        while n_steps <= agent.batch_size:
            observation = env.reset()
            action = env.action_space.sample()
            observation_, reward, done, info = env.step(action)
            agent.remember(observation, action, reward, observation_, done)
            n_steps += 1
        agent.learn()    
        agent.load_models()
        evaluate = True
    else:
        evaluate = False

    for i in range(n_games):
        observation = env.reset()
        print('returned observation: ')
        print(observation)
        done = False
        score = 0
        while not done:
            print(observation)
            action = agent.choose_action(observation, evaluate)
            action_m = tf.make_tensor_proto(action)
            control_action = tf.make_ndarray(action_m)
            print('control action:')
            print(control_action)
            observation_, reward, done, info = env.step(control_action)
            print('reward: ')
            print(reward)
            score += reward
            agent.remember(observation, action, reward, observation_, done)
            if not load_checkpoint:
                agent.learn()
            observation = observation_

        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            best_score = avg_score
            if not load_checkpoint:
                agent.save_models()

        print('episode ', i, 'of ', n_games, 'score %.1f' % score, 'avg score %.1f' % avg_score)

    if not load_checkpoint:
        x = [i+1 for i in range(n_games)]
        plot_learning_curve(x, score_history, figure_file)


if __name__ == '__main__':
    main()