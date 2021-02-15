import rclpy

import gym

from time import sleep

print(gym.__file__)


def main(args=None):

    rclpy.init(args=args)

    env = gym.make('gym_parallel_robot:ParallelRobot-v0')

    loop = True

    while loop:
        obs_, reward, done, info = env.step([0.8, 0.8, 0.8, 0.8])

    env.close()


if __name__ == '__main__':
    main()