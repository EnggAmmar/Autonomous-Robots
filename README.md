# Autonomous-Robots

Starter Webots project for the Autonomous Robots Modularbeit.

## Current status

This repository contains a starter Python controller and three **training worlds**. These worlds are not the official final environments from the course. Replace the files in `worlds/` with the official Moodle worlds when they are provided.

## Structure

```text
controllers/rosbot_navigation/
  rosbot_navigation.py   Main Webots controller
  config.py              Tuning parameters and run mode
  robot_interface.py     Webots device wrapper
  mapping.py             Occupancy grid
  astar.py               A* planner
  frontier.py            Frontier selection
  dwa.py                 Dynamic Window Approach planner
  vision.py              Color detection
  coordinates.py         World/grid conversion
  logger.py              Timing/log output

protos/
  SimpleRosBot.proto     Simple differential-drive training robot

worlds/
  Autonomous Robot_1.wbt
  environment_1.wbt
  environment_2.wbt
  environment_3.wbt
```

## Python setup

From the project root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## First Webots test

1. Open Webots.
2. Open `worlds/environment_1.wbt`.
3. The robot controller is already set to `rosbot_navigation` in the training worlds.
4. Open `controllers/rosbot_navigation/config.py`.
5. Start with:

```python
MODE = "DEVICE_SCAN"
```

6. Press Play in Webots.
7. Check the Webots console. It should print the available robot devices.

## Second test

After the device list appears, change:

```python
MODE = "MOTOR_TEST"
```

The robot should move slowly, turn, and stop.

## Navigation mode

After the motor test works, change:

```python
MODE = "RUN"
```

This activates the starter navigation stack:

- camera color detection
- occupancy grid mapping
- frontier exploration
- A* path planning
- DWA local planning
- blue pillar first, yellow pillar second

## Important assignment note

The course instructions allow external help/tools only with proper referencing. If this starter code is used for submission, document it clearly and make sure you understand, test, and modify the implementation yourself.
