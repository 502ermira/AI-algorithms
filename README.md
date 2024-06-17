# Pathfinding in Gridworld

## Overview

This repository contains a Python project that implements various pathfinding algorithms in a gridworld environment. The objective is to find the optimal path between two points in a grid with obstacles using different algorithms.

## Algorithms Implemented

- **`A*` Algorithm**: Utilizes heuristics to efficiently find the shortest path.
- **Genetic Algorithm**: Inspired by natural selection, evolves solutions over generations.
- **Ant Colony Optimization (ACO)**: Based on the behavior of ants searching for food.
- **Depth-First Search (DFS)**: Explores as far as possible along each branch before backtracking.
- **Q-Learning**: A model-free reinforcement learning algorithm.
- **SARSA (State-Action-Reward-State-Action)**: An on-policy reinforcement learning algorithm.

## Algorithm Details
- **`A*` Algorithm**
The A* algorithm uses a combination of cost from the start node and a heuristic to estimate the cost to the goal. The evaluation function f(n) = g(n) + h(n) guides the search process.

- **Genetic Algorithm**
This algorithm starts with a randomly generated population of paths and applies crossover and mutation operations to evolve the population towards optimal solutions over several generations.

- **Ant Colony Optimization (ACO)**
Simulates ant foraging behavior by using pheromone trails to probabilistically find the shortest paths. Pheromone levels are updated based on the quality of the solutions found.

- **Depth-First Search (DFS)**
DFS explores paths to their maximum depth before backtracking, useful for exploring all possible paths but can be inefficient in large grids.

- **Q-Learning and SARSA**
These reinforcement learning algorithms learn optimal policies for navigating the gridworld by maximizing cumulative rewards through interaction with the environment. Q-Learning is off-policy while SARSA is on-policy.

### Steps to be followed to run various algorithms to find the optimal path:
----
This Project code is available in the src folder of the project repository. 
##### Pre-requisites: 
- Install Python
- Install dependencies like ***tkinter, matplotlib.pyplot, scipy.ndimage.filters, pylab***
##### Algorithms Execution: 
- Run A-Star Algorithm by going to main folder and executing below command in command prompt.
    ```
    python RunAStar.py
    ```
- Run Genetic Algorithm by going to main folder and executing below command in command prompt.
    ```
    python RunGenetic.py
    ```
- Run Q-Learning Algorithm by going to main folder and executing below command in command prompt.
    ```
    python  RunRL.py
    ```
- Run SARSA Algorithm by going to main folder and executing below command in command prompt.
    ```
    python  RunSARSA.py
    ```

## Results

The experiments evaluated the algorithms' efficiency and optimality across different grid sizes and heuristic functions.

### A* Algorithm

The A* algorithm was tested using different heuristic functions on a fixed 40x40 grid with random obstacles. The heuristics evaluated included a constant heuristic, Euclidean distance, Manhattan distance, and more complex combinations. The results demonstrated that the Manhattan distance heuristic was the most efficient and optimal due to the grid's constraints against diagonal movements. Other heuristics either increased the number of visited nodes or did not provide optimal paths.

**Performance Summary:**
- **Constant Heuristic:** 81 steps, 1164 nodes visited
- **Euclidean Distance:** 81 steps, 1086 nodes visited
- **Manhattan Distance:** 81 steps, 625 nodes visited
- **\( x^2 + y^2 \):** 85 steps, 170 nodes visited
- **2 * (X + Y):** 85 steps, 210 nodes visited

### Genetic Algorithm

The Genetic Algorithm (GA) was tested on various grid sizes, ranging from 5x5 to larger grids. The algorithm's performance was measured in terms of convergence rates and the number of iterations required to find an optimal path. The experiments showed that as the grid size increased, the number of iterations needed for convergence also increased, but the growth was sub-linear. This indicates that the algorithm efficiently scales with the grid size, although larger grids naturally require more iterations due to increased complexity.

**Key Observations:**
- Smaller grids like 5x5 required fewer iterations for convergence.
- Larger grids showed a sub-linear increase in iterations needed, suggesting efficient scalability.
- The mutation rate and population size adjustments significantly affected the convergence rate and solution quality.

### Ant Colony Optimization (ACO)

The ACO algorithm was tested to simulate the foraging behavior of ants. It uses pheromone trails to probabilistically find the shortest paths. Pheromone levels were updated based on the quality of the solutions found. The ACO showed promising results, particularly in dynamic environments where paths needed to be frequently recalculated.

**Key Observations:**
- ACO was effective in finding multiple feasible paths, adapting well to changes in the grid.
- Performance improved with increased iterations, but it was more computationally intensive compared to other algorithms.

### Reinforcement Learning (Q-learning and SARSA)

Reinforcement Learning algorithms were also implemented and tested. These algorithms allow the agent to learn the optimal path through interaction with the environment, adjusting based on rewards received from different actions. The results highlighted the differences between Q-learning and SARSA in terms of convergence rates and the quality of the paths found. Both methods showed promising results in learning optimal paths, but specific performance metrics were not detailed in the document.

### Discussion

Overall, the experiments demonstrated that heuristic choice significantly impacts the A* algorithm's efficiency. The Manhattan distance heuristic was particularly effective in the given grid setup. The Genetic Algorithm showed good scalability with grid size, making it a robust choice for larger environments. ACO was effective in dynamic environments, though computationally intensive. Reinforcement Learning methods proved capable of learning optimal paths, with Q-learning and SARSA offering different strengths in pathfinding performance.

These results provide valuable insights into the strengths and limitations of different pathfinding approaches, contributing to a better understanding of their application in various scenarios.
