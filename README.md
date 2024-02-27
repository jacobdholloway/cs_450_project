# Created by Jacob Holloway
# CS 450 Intro to AI
# San Diego State University Spring 2024

# Assignment One: TimidAgent in myAgents.py

## Overview
The `TimidAgent` class is designed for the Pacman game to navigate while avoiding close, non-scared ghosts. It implements strategies for evasion and movement based on ghost proximity and their states.

## Implementation

- **`__init__()`**: Initializes the `TimidAgent` without additional parameters.
- **`inDanger(pacman, ghost, dist)`**: Evaluates if Pacman is in danger based on ghost proximity and status, guiding movement decisions.
- **`getAction(state)`**: Determines Pacman's next move considering legal actions and the ghost states.
- **`decideOnDanger(dangerDirection, legal, heading)`**: Chooses an action when in danger, based on available legal actions and current direction.
- **`followLeftTurnBehavior(heading, legal)`**: Default movement strategy prioritizing left turns, then straight, right turns, or reversing.

## Behavior

- **Avoidance**: Changes direction or turns away when a non-scared ghost is nearby.
- **Movement Strategy**: Prefers left turns, then moves straight, turns right, or reverses if necessary.
- **Safety Checks**: Ensures moves are legal and safe from collisions.

## Usage

Deploy `TimidAgent` in Pacman by setting it as the agent to balance safety with exploration.

# Assignment Two: search.py and explored.py

## Overview

### Graph Search Function

- **Generalized framework** for state exploration using a priority queue.
- **Parameters**: Problem, cost functions, and debug modes.
- **Process**: Initiates from the start state, explores successors, avoids redundant states.
- **Goal Check**: Completes upon reaching a goal state, then reconstructs the path.

### A* Search Function

- Utilizes path cost and heuristic for efficient goal-reaching.
- Implements a priority queue based on estimated costs to the goal.

### Breadth-First Search (BFS) Function

- Explores nodes level by level using a standard queue.
- Prevents revisiting states to avoid redundant searches.

### Uniform Cost Search

- Described as a strategy expanding nodes based on cumulative costs (details not provided).

## Additional Details

- **SearchNode Class**: Facilitates state tracking and path reconstruction.
- **Utility Functions**: Includes a default heuristic for non-heuristic scenarios.

## Implementation Notes

- **Custom Solutions**: Tailored `SearchNode` class for assignment requirements.
- **Testing and Validation**: Ensured algorithm functionality and correctness.
- **Debugging and Verbosity**: Flags for detailed search process insights.

# Class Overview: Explored

## Implementation

- **Data Structure**: Uses a set named `hashtable` for storing explored states for efficient lookups.
- **Initialization**: Starts with an empty set ready for tracking explored states.

## Methods

- **`exists(state)`**: Checks if a state is already explored.
- **`add(state)`**: Adds a new state to the set, marking it as explored.

## Usage in Search Algorithms

- Ensures efficiency by avoiding redundant state expansions.
- Checks and marks states during exploration to optimize search performance.
