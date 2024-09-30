# Control Workshop code

## Test robot

Arduino code in `arduino/`

## Python environment

You'll need a few things to set up a development environment:

* Install Python using [uv](https://github.com/astral-sh/uv?tab=readme-ov-file#installation).
* [VSCode](https://code.visualstudio.com/download)
* [VSCode Python extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

## Training a control model

Train the model using the notebook in `controlbot_training.ipynb`.

## Running a simulated robot

Run a simulation of the robot: `uv run simulation/simulation.py`

Hold down the `left shift key` to increase the left wheel speed and the `right shift key` to increase the right wheel speed.  Release the keys to decrease the wheel speeds.  Press the `space bar` to stop the robot.  Press `r` to reset the robot to the starting position.

Log data that can be used for training is written to `log.csv` (change in left wheel encoder, change in right wheel encoder, change in heading angle (in radians)).
