import argparse
import stable_baselines3
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
import os
from stable_baselines3.common.env_util import make_vec_env
from rl_env import Sim
import numpy as np
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
ENVIROMENT_CLASSES = {
    'real': Sim
}


def make_env(use_gui, discrete, env_class_name, cnn_policy, play_only):
    env_class = ENVIROMENT_CLASSES[env_class_name]
    return Monitor(env_class(gui=use_gui, discrete=discrete, cnn_policy=cnn_policy, play_only=play_only))


def main():
    parser = argparse.ArgumentParser(description='Gripper DRL.')
    parser.add_argument('--algorithm', default='PPO', type=str, help='Algorithm')
    parser.add_argument('--gui', default=False, action='store_true')
    parser.add_argument('--model', default='', type=str, help='Path to the model')
    parser.add_argument('--play_only', default=False, action='store_true')
    parser.add_argument('--environment', default='real', type=str)
    parser.add_argument('--discrete', default=True, action='store_true')

    parser.add_argument('--cnnpolicy', default=False, action='store_true')
    parser.add_argument('--jobs', default=1, type=int, help='Number of parallel simulations')
    parser.add_argument('--n_steps', default=128, type=int)
    parser.add_argument('--lr', default=0.0001, type=float)
    parser.add_argument('--n_epochs', default=10, type=int)
    parser.add_argument('--batch_size', default=64, type=int)
    parser.add_argument('--gae_lambda', default=0.95, type=float)
    args = parser.parse_args()

    # classification_model = None
    env = make_vec_env(lambda: make_env(args.gui, args.discrete, args.environment, args.cnnpolicy,
                       args.play_only), n_envs=args.jobs, vec_env_cls=SubprocVecEnv)

    # Load DRL algorithm
    drl_algorithm_classes = {'PPO': stable_baselines3.PPO}
    drl_algorithm_class = drl_algorithm_classes[args.algorithm]

    # Initialize model
    if args.model != '':
        # Load model
        model = drl_algorithm_class.load(args.model, env=env, custom_objects={
                                         "learning_rate": args.lr, "batch_size": args.batch_size, "n_epochs": args.n_epochs, "n_steps": args.n_steps})
        print('load model successful')
    else:
        algorithm_args = {}
        if args.cnnpolicy is True:
            policy = 'CnnPolicy'
        else:
            policy = 'MultiInputPolicy'

        if args.n_steps is not None:
            algorithm_args['n_steps'] = args.n_steps

        # Model from scratch
        model = drl_algorithm_class(
            policy,
            env,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            learning_rate=args.lr,
            gae_lambda=args.gae_lambda,
            verbose=1,
            tensorboard_log='logs/tb/all3',
            **algorithm_args
        )

    # Play or learn
    if args.play_only:
        # Play
        number_of_episodes = 0
        full_reward = []
        obs = env.reset()
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            print(reward)
            if done:
                number_of_episodes += 1
                obs = env.reset()
    else:
        # Learn
        model.learn(
            total_timesteps=100_000_000_000_000,
            callback=[
                CheckpointCallback(
                    save_freq=max(args.n_steps*4, 1),
                    save_path='./logs/models/rss',
                ),
            ]
        )
        model.save('final')


if __name__ == '__main__':
    main()
