import gym

from time import sleep


def main(args=None):


    env = gym.make('gym_parallel_robot:ParallelRobot-v0')

    n_episodes = 20
    i = 0

    for i in range(0, n_episodes):
        print('episode number' + str(i))
        env.reset()
        loop = True

        while loop:
            obs_, reward, done, info = env.step([0.8, 0.8, 0.8, 0.8])
            print(reward)
            print(obs_)
            if done == True:
                loop = False
            
        i = i+1

    env.close()


if __name__ == '__main__':
    main()