# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).



# Programming assignment Affadit

# Single programmer:
# I promise that the attached assignment is my own work. I recognize that should this not
# be the case, I will be subject to penalties as outlined in the course syllabus. Jacob Holloway



# Created by Jacob Holloway
# Made for CS 450 Intro to AI
# search.py is part of Assignment 2 part 2
# Worked on search.py and explore.py
# Any part I worked on will have Jacob Holloway next to it


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def genericSearch(problem, g, h):
    state = problem.getStartState()
    goal = problem.goal


##################################
# ASSIGNMENT TWO CS 450 PART TWO #
##################################


##########################
#      DFS THIS ONE      #
#   THIS ONE IS WORKING  #
########################## 
    
# Jacob Holloway Assignment 2 part 2

def depthFirstSearch(problem):
    # Import necessary classes
    from util import Stack

    # Initialize the stack with the starting point
    start = problem.getStartState()
    stack = Stack()
    stack.push((start, [], 0))  # Each entry is (currentState, actionsToHere, costToHere)

    # Keep track of visited nodes
    visited = set()

    while not stack.isEmpty():
        currentState, actions, currentCost = stack.pop()

        # If this state is the goal, return the actions that got us here
        if problem.isGoalState(currentState):
            return actions

        # If the state has not been visited, mark it as visited
        if currentState not in visited:
            visited.add(currentState)

            # Explore successors
            for successor, action, stepCost in problem.getSuccessors(currentState):
                # Add successor to the stack if not already visited
                if successor not in visited:
                    # Append the current action to the list of actions
                    newActions = actions + [action]
                    stack.push((successor, newActions, currentCost + stepCost))

    # Return empty list if no path found
    return []

##################################
# ASSIGNMENT TWO CS 450 PART TWO #
##################################


##########################
#      BFS THIS ONE      #
#   THIS ONE IS WORKING  #
########################## 

# Jacob Holloway Assignemnt 2 part 2

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # Import necessary classes
    from util import Queue

    # Initialize the queue with the starting point
    start = problem.getStartState()
    queue = Queue()
    queue.push((start, []))  # Each entry is (currentState, actionsToHere)

    # Keep track of visited nodes to avoid cycles
    visited = set()

    while not queue.isEmpty():
        currentState, actions = queue.pop()

        # If this state is the goal, return the actions that got us here
        if problem.isGoalState(currentState):
            return actions

        # If the state has not been visited, mark it as visited
        if currentState not in visited:
            visited.add(currentState)

            # Explore successors
            for successor, action, _ in problem.getSuccessors(currentState):
                # Check if the successor has already been visited
                if successor not in visited:
                    # Add successor to the queue with the corresponding action
                    newActions = actions + [action]
                    queue.push((successor, newActions))

    # Return empty list if no path found
    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

##################################
# ASSIGNMENT TWO CS 450 PART TWO #
##################################

##########################
#     A* SEARCH ONE      #
#  THIS ONE IS WORKING   #
########################## 

# Jacob Holloway Assignment 2 part 2

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # Import necessary classes
    from util import PriorityQueue

    # Initialize the priority queue with the starting point
    start = problem.getStartState()
    pq = PriorityQueue()
    pq.push((start, [], 0), 0)  # Each entry is (currentState, actionsToHere, costToHere)

    # Keep track of visited nodes to avoid cycles and unnecessary work
    visited = set()

    while not pq.isEmpty():
        currentState, actions, currentCost = pq.pop()

        # If this state is the goal, return the actions that got us here
        if problem.isGoalState(currentState):
            return actions

        # If the state has not been visited, mark it as visited
        if currentState not in visited:
            visited.add(currentState)

            # Explore successors
            for successor, action, stepCost in problem.getSuccessors(currentState):
                if successor not in visited:
                    # Calculate new costs
                    newCost = currentCost + stepCost
                    # Total cost is the cost-to-here plus the heuristic estimate to the goal
                    totalCost = newCost + heuristic(successor, problem)
                    # Add successor to the priority queue with the new total cost
                    pq.push((successor, actions + [action], newCost), totalCost)

    # Return empty list if no path found
    return []

##################################
# ASSIGNMENT TWO CS 450 PART TWO #
##################################

# Made this class based on game needing to track and search. 
# Looked for SearchNode in search.py but could not find so I made my own.

# Jacob Holloway Assignment 2 part 2

class SearchNode:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        """
        Create a new SearchNode.

        :param state: The current state represented by this node.
        :param parent: The parent SearchNode that generated this node.
        :param action: The action taken to get from the parent node to this node.
        :param path_cost: The total cost to reach this node from the start node.
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __lt__(self, other):
        """
        This method is necessary for the PriorityQueue to compare nodes.
        It defines a node to be less than another node if its path cost is lower.
        """
        return self.path_cost < other.path_cost

    def get_path(self):
        """
        Reconstruct the path from the start node to this node (used when the goal is found).

        :return: A list of actions to get from the start node to this node.
        """
        node, actions = self, []
        while node.parent is not None:
            actions.append(node.action)
            node = node.parent
        actions.reverse()  # The actions are added from the goal to the start so we need to reverse them
        return actions



##################################
# ASSIGNMENT TWO CS 450 PART TWO #
##################################
    
# Jacob Holloway
# Asked to create this as per Assignment 2 part 2

def graph_search(problem, g, h, verbose=False, debug=False):
    from util import PriorityQueue

    # Define the initial state and the frontier using a priority queue
    start_node = SearchNode(problem.getStartState())
    frontier = PriorityQueue()
    frontier.push(start_node, g(start_node) + h(start_node, problem))

    # Initialize an empty set for explored states
    explored = set()

    # Loop until there are no more nodes in the frontier
    while not frontier.isEmpty():
        # Pop a node from the frontier
        node = frontier.pop()

        # Check for goal state
        if problem.isGoalState(node.state):
            # Construct and return the solution path
            return reconstruct_path(node)

        # Mark node as explored
        explored.add(node.state)

        # Expand the node and add all unexplored successors to the frontier
        for successor, action, cost in problem.getSuccessors(node.state):
            child_node = SearchNode(successor, node, action, node.path_cost + cost)

            # Check if we have already explored this state or if it's already in the frontier with higher cost
            if successor not in explored and (child_node not in frontier.heap or g(child_node) < frontier.getPriority(child_node)):
                frontier.update(child_node, g(child_node) + h(child_node, problem))

        # Optional: Print debugging information
        if verbose or debug:
            print("Frontier:", frontier.heap)
            print("Explored:", explored)

    # Return failure if no solution was found
    return None

def reconstruct_path(node):
    """Reconstruct a path to the goal by following parent pointers."""
    actions = []
    while node.parent:
        actions.append(node.action)
        node = node.parent
    actions.reverse()  # The actions are in goal-to-start order, so reverse them
    return actions




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
