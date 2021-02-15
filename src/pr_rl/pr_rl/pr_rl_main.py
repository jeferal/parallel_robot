import rclpy
import sys

import gym

print(gym.__file__)


def main(args=None):

    rclpy.init(args=args)
    print(sys.path)

    env = gym.make('gym_parallel_robot:ParallelRobot-v0')
    env.close()


if __name__ == '__main__':
    main()