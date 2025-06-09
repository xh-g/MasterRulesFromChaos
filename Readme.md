# MasterRulesFromChaos

This repository provides code for Master Rules from Chaos: Learning to Reason, Plan, and Interact from Chaos for Tangram Assembly.

## Setup

1. Install the required Python packages (PyTorch, torchvision, stable-baselines3, pybullet, numpy and matplotlib). A virtual environment is recommended.
2. Some scripts depend on additional hardware specific packages (e.g. `pyrealsense2` for the L515 camera). These are only needed when running on the real robot.

## Dataset Generation

To speed up, random patterns for the tangram are pre-generated using the code in the folder util_code

## Training

`policy_train_rl.py` launches training with stable-baselines3. A typical invocation is:

```bash
python policy_train_rl.py --algorithm PPO --jobs 4 --n_steps 128 --batch_size 64
```

The script accepts many arguments for parallel environments, learning rate, number of epochs and more.

## Testing

- **Reinforcement Learning:**
  - Simulation with random layouts: `python test_in_sim_random_pattern.py`
  - Human-designed layouts: `python test_in_sim_human_pattern_auto.py`
  - Real robot with L515 depth camera: `python test_in_real_with_l515.py` (requires hardware and `pyrealsense2`).

Pretrained RL policies (`4-1.zip`, `5-1.zip`, etc.) can be loaded using `stable_baselines3.PPO.load()` as demonstrated in the test scripts.

## License

This project is licensed under the Apache 2.0 License. See `LICENSE` for details.
