import sys
import os
import numpy as np
import gymnasium as gym
from gymnasium import spaces

sys.path.append(os.path.abspath("build/python"))
import robot_env_cpp


class RobotGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, map_path, robot_radius=10, add_noise=False, max_steps=400):
        super().__init__()

        

        self.core = robot_env_cpp.RobotEnv(
            map_path,
            robot_radius,
            add_noise,
            max_steps
        )
        self.core.setRender(True)
        self.core.setRenderDelayMs(1)

        obs0 = self.core.reset(0)
        obs_dim = len(obs0)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )
        self.action_space = spaces.Discrete(4)

    def reset(self, *, seed=None, options=None):
        if seed is None:
            seed = 0
        obs = self.core.reset(int(seed))
        obs = np.asarray(obs, dtype=np.float32)

        info = {
            "dist_to_goal": float(self.core.getDistToGoal())
        }
        return obs, info

    def step(self, action):
        out = self.core.step(int(action))
        obs = np.asarray(out.obs, dtype=np.float32)

        info = {
            "dist_to_goal": float(self.core.getDistToGoal())
        }

        return (
            obs,
            float(out.reward),
            bool(out.terminated),
            bool(out.truncated),
            info,
        )