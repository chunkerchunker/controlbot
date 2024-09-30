import gymnasium as gym
import pygame
import numpy as np
from datetime import datetime

import controlbot_gym

# create a simulation environment
env = gym.make("controlbot_gym/ControlBot-v0", render_mode="human")
env.reset()

vl = 0  # left wheel velocity
vr = 0  # right wheel velocity
dv = 0.1  # velocity increment
vmin = 0  # minimum velocity
vmax = 10  # maximum velocity
last_theta = 0  # last known angle

# open a csv file for logging
with open(f"logs/log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv", "w") as f:

    quit = False
    while not quit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit = True

        # detect key presses
        keys = pygame.key.get_pressed()

        # reset env on "r"
        if keys[pygame.K_r]:
            env.reset()
            vl = 0
            vr = 0

        # stop wheels on "space"
        if keys[pygame.K_SPACE]:
            vl = 0
            vr = 0

        # increment left/right wheel velocity while left/right shift is pressed; decrement otherwise
        l = keys[pygame.K_LSHIFT]
        r = keys[pygame.K_RSHIFT]
        vl = min(max(vl + (dv if l else -dv), vmin), vmax)
        vr = min(max(vr + (dv if r else -dv), vmin), vmax)

        # send action to the environment
        action = {
            "v_l": np.array([vl], dtype=np.float32),
            "v_r": np.array([vr], dtype=np.float32),
        }
        observation, reward, terminated, truncated, info = env.step(action)
        theta = observation["theta"][0]
        d_theta = theta - last_theta
        last_theta = theta

        # log actions & observed state
        if vl != 0 or vr != 0:
            f.write(f"{vl},{vr},{d_theta}\n")

    env.close()
