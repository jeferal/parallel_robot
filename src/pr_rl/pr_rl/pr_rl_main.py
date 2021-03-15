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
            n_actions=env.action_space.shape[0], alpha=3e-7, beta=5e-7,
            gamma=0.99, max_size=10000000, tau=0.005, 
            batch_size=64, noise=0.05, fc1=700, fc2=500)
    
    n_games = 5000

    figure_file = 'plots/pr_avg_score.png'

    best_score = env.reward_range[0]
    score_history = []
    load_checkpoint = False
    evaluate = False

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

    for i in range(n_games):
        observation = env.reset()
        done = False
        score = 0
        while not done:
            action = agent.choose_action(observation, evaluate)
            action_m = tf.make_tensor_proto(action)
            control_action = tf.make_ndarray(action_m)
            print(control_action)
            observation_, reward, done, info = env.step(control_action)
            print(reward)
            score += reward
            agent.remember(observation, action, reward, observation_, done)
            if not load_checkpoint:
                agent.learn()
            observation = observation_

        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if i%100 == 0:
            best_score = avg_score
            print('--- Saving weights ---')
            if not load_checkpoint:
                agent.save_models()

        print('episode ', i, '/', n_games, 'score %.1f' % score, 'avg score %.1f' % avg_score)

    x = [i+1 for i in range(n_games)]
    plot_learning_curve(x, score_history, figure_file)


if __name__ == '__main__':
    main()