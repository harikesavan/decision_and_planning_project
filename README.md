Decision and Path Planning Module
=============================


In this project you will be tasked with implementing two critical components of a traditional hierarchical planner: the Behaviour Planner and the Motion Planner. These two components will work together to achieve the following goals

- Avoid collisions with stationary objects.

- Manage all types of junctions.

- Maintain the centre line of the driving lane.

To achieve these goals, you need to implement the following processes:

- Behavioural planning logic using Finite State Machines (FSM).

- Collision checking with stationary objects and Cost function implementation.

- Path and trajectory generation using cubic spirals.

- Selection of the best trajectory by evaluation of a cost function. This cost function will mainly perform a collision check and a proximity check to increase the cost as the vehicle approaches or collides with objects, while maintaining a bias to stay closer to the lane centreline.

## Installation

We have provided a set of instructions to install all the necessary requirements for this course. To ensure a seamless installation process, please follow the instructions that correspond to your operating system.

* [windows](./Installation/Installation_win.md)
* [linux](./Installation/Installation_linux.md)

## Project Instructions

To make the task easier, we have already implemented some parts of the code. Your job is to complete the TODOS specified in the following files according to the instructions given.

#### General Parameters

For this you will have to complete the TODOS in [PlanningParams](./planners/PlanningParams.py)

1. TODO - Num of Path (goals).
2. TODO - Num of points in the spiral.

#### Behavioral Planning Logic

For this part of the project, your task is to define the states and logic of an FSM that handles static objects and intersections that require a 5-second stop. You will need to implement the necessary transitions, which are:

1. LANE FOLLOWING -> DECELERATION STOP
2. DECELERATION STOP -> STOP
3. STOP -> LANE FOLLOWING

For this you will have to complete the TODOS in [BehavioralPlannerFSM.py](./planners/BehavioralPlannerFSM.py)

1. TODO - Lookahead
2. TODO - Goal behind the stopping point
3. TODO - Goal speed at stopping point
4. TODO - Goal speed in nominal state
5. TODO - Maintain the same goal in DELERATION STOP
6. TODO - Calculate Speed
7. TODO - Use distance rather than speed
8. TODO - Move to Stopped state
9. TODO - Maintain the same goal when stopped
10. TODO - Move to FOLLOW LANE

#### Collision checking with stationary objects and cost function implementation

In this section of the project, your task is to implement a circle-based collision checking algorithm. This algorithm will be used to assess each trajectory generated against an objective function and select the most optimal one. The objective function aims to assign a score to each path, with poor paths that are in collision or too close to static obstacles receiving low scores, while high scores are assigned to paths that are closer to the center-line of the global path.

For this you will have to complete the TODOS in [cost_function.py](./planners/cost_functions.py)

1. TODO - Circle placement.
2. TODO - Distance from circles to obstacles/actor.
3. TODO - Distance between last point on spiral and main goal.

#### Path generation using cubic spirals

For this segment of the project, your objective is to determine where the goal point should be located based on the desired behavior (Behavior Planner). Ideally, the goal should be positioned at the center of the lane, at a distance of "d" ahead of the ego vehicle. However, since it's uncertain whether a feasible path exists between these two points, you will need to leverage the road structure and choose laterally offset goals from the ideal location. This will generate alternative paths, which will be evaluated later on.

For this you will have to complete the TODOS in [MotionPlanner.py](./planners/MotionPlanner.py)

1. TODO - Calculate perpendicular direction.
2. TODO - Calculate offset goal location.

#### Velocity profile generation

In this part of the project, your task is to create a velocity profile for each of the states that the vehicle can be in. When in the "Follow_lane" state, you will need to adjust the speed either up or down to maintain a specified speed target. In the "decel_to_stop" state, you will need to generate a profile that allows for a smooth deceleration to the stop line. The order of precedence for handling these cases is stop sign handling followed by nominal lane travel.

For this you will have to complete the TODOS in [VelocityProfileGenerator.py](./planners/VelocityProfileGenerator.py)

1. TODO - Calculate distance.
2. TODO - Calculate speed.
3. TODO - Calculate deceleration trajectory. 
4. TODO - Calculate nominal trajectory.

### Testing

For your convinience we have prepared several unittest in the folder [tests](./Project/tests/) so you can check if your algorithm is working properly. To run all the test, type in your terminal:

<code>pytest tests -p no:warnings -s</code>

Also you can run each test individually by typing:

<code>pytest tests/test_{module}.py -p no:warnings -s</code>

Before running the whole project, it's important to ensure that you have passed all the tests. This will help you catch any errors or issues early on, and ensure that your project is working properly.

## Running whole project

After completition of the code, you can check that the project is running by running the simulator. For this you need to:

1. Run CARLA 
2. Run [SimulatorAPI.py](./SimulatorAPI.py)

<code>python SimulartorAPI.py</code>

You should see a car moving in the simulator. The project will be considered correct if the car is able to:

- Avoid the 3 obstacles.
- Stop on each juctions
- Continue moving seamesly after the second junction.

## Submission Template


The submission must be done in a zip file containing the implemented code and a report in a **PDF** with the following specifications:

### Project Overview

This section should contain a brief description of the project and what you are trying to achieve.

### Set up

This section should contain a brief description of the steps to follow to run the code. There must be a detailed explanation of how to run the implemented code so we can see run them on our local machine.

### Behavioral Planning

This section should describe the behavioral planning using FSM, specifically:

- Brief explanation of what are FSM. 
- States of the FSM.
- Conditions and transitions between states.
- A graph with the FSM is preferred.

### Path and trajectory generation using cubic spirals

This section should describe the how cubic spirals work and how trajectories, specifically::

- Brief explanation of Cubic Spirals. 
- Explanation of why goal offset are necessary.
- Explanation of goal offset calculations.
- Explanation of collision checking.
- Explanation of spiral cost function.

### Velocity profile generation

This section should describe how the velocity profile generator works, specifically:

- Brief explanation of velocity profile generation.
- How do you calculate the profiles. 
- How do you calculate distances or velocity for each point (depends on the case).

### Analysis

This section should include a extensive analysis of the whole program.

Please note that your report should be in PDF format, and your submission should include the implemented code and any additional materials necessary for us to replicate your results.

## Disclaimer

If you are taking the course Introduction to Autonomous Vehicles and encounter any issues with the setup or tasks, please reach out to your instructors as soon as possible. They are there to help you and want to ensure you have a smooth and successful learning experience. Don't hesitate to ask them any questions you may have, and they will provide you with the support you need.


Good luck with your task! 🚀👍