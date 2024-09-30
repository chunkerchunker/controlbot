from typing import Optional, Union
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from gymnasium.error import DependencyNotInstalled

from gymnasium.envs.registration import register

register(
    id="controlbot_gym/ControlBot-v0",
    entry_point="controlbot_gym:ControlBotEnv",
)


class ControlBotEnv(gym.Env[np.ndarray, Union[int, np.ndarray]]):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 10,
    }

    def __init__(self, render_mode: Optional[str] = None):
        max_speed = 10.0
        self.field_size = 600
        self.track_width = 5.0

        self.action_space = gym.spaces.Dict(
            {
                "v_l": spaces.Box(-max_speed, max_speed, dtype=np.float32),
                "v_r": spaces.Box(-max_speed, max_speed, dtype=np.float32),
            }
        )
        self.observation_space = gym.spaces.Dict(
            {
                "x": spaces.Box(0, self.field_size, dtype=np.float32),
                "y": spaces.Box(0, self.field_size, dtype=np.float32),
                "dl": spaces.Box(-max_speed, max_speed, dtype=np.float32),
                "dr": spaces.Box(-max_speed, max_speed, dtype=np.float32),
                "theta": spaces.Box(-np.pi, np.pi, dtype=np.float32),
            }
        )

        self.render_mode = render_mode
        self.screen = None
        self.clock = None
        self.isopen = True

    def step(self, action):
        assert self.action_space.contains(
            action
        ), f"{action!r} ({type(action)}) invalid"

        self.vl = action["v_l"][0]
        self.vr = action["v_r"][0]

        self.x, self.y, self.theta = self.diffdrive(
            self.x,
            self.y,
            self.theta,
            self.vl,
            self.vr,
            1,
            self.track_width,
        )

        terminated = bool(
            self.x < 0
            or self.x > self.field_size
            or self.y < 0
            or self.y > self.field_size
        )

        reward = 0.0 if terminated else 1.0

        if self.render_mode == "human":
            self.render()

        return self._get_obs(), reward, terminated, False, {}

    def _get_obs(self):
        return {
            "x": np.array([self.x], dtype=np.float32),
            "y": np.array([self.y], dtype=np.float32),
            "dl": np.array([self.vl], dtype=np.float32),
            "dr": np.array([self.vr], dtype=np.float32),
            "theta": np.array([self.theta], dtype=np.float32),
        }

    def reset(self, *, seed: Optional[int] = None, **kwargs):
        super().reset(seed=seed)
        self.vl = 0.0
        self.vr = 0.0
        self.x = self.field_size / 2
        self.y = self.field_size / 2
        self.theta = 0.0

        if self.render_mode == "human":
            self.render()

        return self._get_obs(), {}

    def render(self):
        if self.render_mode is None:
            assert self.spec is not None
            gym.logger.warn(
                "You are calling render method without specifying any render mode. "
                "You can specify the render_mode at initialization, "
                f'e.g. gym.make("{self.spec.id}", render_mode="rgb_array")'
            )
            return

        try:
            import pygame
            from pygame import gfxdraw
        except ImportError as e:
            raise DependencyNotInstalled("pygame is not installed") from e

        if self.screen is None:
            pygame.init()
            if self.render_mode == "human":
                pygame.display.init()
                self.screen = pygame.display.set_mode(
                    (self.field_size, self.field_size)
                )
            else:  # mode == "rgb_array"
                self.screen = pygame.Surface((self.field_size, self.field_size))
        if self.clock is None:
            self.clock = pygame.time.Clock()

        self.surf = pygame.Surface((self.field_size, self.field_size))
        self.surf.fill((255, 255, 255))

        car = get_triangle(self.x, self.y, 10, self.theta)
        pygame.draw.polygon(self.surf, (255, 0, 0), car)

        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        if self.render_mode == "human":
            pygame.event.pump()
            self.clock.tick(self.metadata["render_fps"])
            pygame.display.flip()

        elif self.render_mode == "rgb_array":
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2)
            )

    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def diffdrive(
        self,
        x: np.float32,
        y: np.float32,
        theta: np.float32,
        v_l: np.float32,
        v_r: np.float32,
        t: int,  # time
        l: np.float32,  # track_width
    ):
        # straight line
        if v_l == v_r:
            theta_n = theta
            x_n = x + v_l * t * np.cos(theta)
            y_n = y + v_l * t * np.sin(theta)

        # circular motion
        else:
            # Calculate the radius
            R = l / 2.0 * ((v_l + v_r) / (v_r - v_l))

            # computing center of curvature
            ICC_x = x - R * np.sin(theta)
            ICC_y = y + R * np.cos(theta)

            # compute the angular velocity
            omega = (v_r - v_l) / l

            # computing angle change
            dtheta = omega * t

            # forward kinematics for differential drive
            x_n = np.cos(dtheta) * (x - ICC_x) - np.sin(dtheta) * (y - ICC_y) + ICC_x
            y_n = np.sin(dtheta) * (x - ICC_x) + np.cos(dtheta) * (y - ICC_y) + ICC_y
            theta_n = theta + dtheta

        return x_n, y_n, theta_n


def rotate_point(x, y, cx, cy, theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    dx = x - cx
    dy = y - cy
    new_x = cx + cos_theta * dx - sin_theta * dy
    new_y = cy + sin_theta * dx + cos_theta * dy
    return new_x, new_y


def get_triangle(x, y, side_length, theta):
    # Calculate height of the equilateral triangle
    height = 1.5 * side_length

    # Define the three points of the equilateral triangle centered at the base
    p1 = (x, y - side_length / 2)  # Left base point
    p2 = (x, y + side_length / 2)  # Right base point
    p3 = (x + height, y)  # Top point

    # Rotate points around the center of the base
    center_x = x
    center_y = y + height / 3  # Center of the triangle to rotate around

    p1_rotated = rotate_point(p1[0], p1[1], center_x, center_y, theta)
    p2_rotated = rotate_point(p2[0], p2[1], center_x, center_y, theta)
    p3_rotated = rotate_point(p3[0], p3[1], center_x, center_y, theta)

    return [p1_rotated, p2_rotated, p3_rotated]
