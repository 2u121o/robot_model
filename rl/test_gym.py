from RobotGymEnv import RobotGymEnv

env = RobotGymEnv("../map_new_obs.png", robot_radius=10, add_noise=False, max_steps=300)
env.core.setRender(True)
env.core.setRenderDelayMs(1)

for ep in range(30):
    obs, info = env.reset(seed=ep + 1)
    
    total = 0.0

    for t in range(300):
        action = env.action_space.sample()

        obs, reward, terminated, truncated, info = env.step(action)
        total += reward

        if terminated or truncated:
            print(f"EP {ep} finished at step {t+1}, total reward {total:.3f}")
            break