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
        # This is the wrapper function that fits the expected Berkeley interface

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

class SearchNode:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        return [self.child_node(problem, action) for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        next_node = SearchNode(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __getitem__(self, key):
        if hasattr(self, key):
            return getattr(self, key)
        else:
            raise KeyError(f"'SearchNode' object has no attribute '{key}'")

    def __setitem__(self, key, value):
        setattr(self, key, value)

    # def __eq__(self, other):
    #     return isinstance(other, )) and self.state == other.state

    def __hash__(self):
        return hash(self.state)



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
        
def aStarSearch(problem, heuristic=nullHeuristic):
    return AStarSearch.search(problem, heuristic)
    
def breadthFirstSearch(problem):
    # This conforms to the expected interface for the Pacman framework
    return BreadthFirstSearch.breadthFirstSearch(problem)
    
def depthFirstSearch(problem):
    # This conforms to the expected interface for the Pacman framework
    return DepthFirstSearch.depthFirstSearch(problem)

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

class DepthFirstSearch:
    
    @classmethod
    def g(cls, node):
        """
        Returns the cost to reach the Node node as defined by the search algorithm being implemented.
        For standard DFS, this might simply return the depth of the node, as DFS does not prioritize paths based on cost.
        """
        # Assuming 'node' is a tuple where the total cost to reach the node is the third element
        return node.depth  # Or return node.depth if you have a Node class with a 'depth' attribute

    @classmethod
    def h(cls, node, problem):
        """
        Returns the heuristic cost to the goal as defined by the search algorithm being implemented.
        For DFS, which is uninformed, this is typically 0 as it does not utilize a heuristic.
        """
        # Since DFS doesn't use heuristics, we just return 0
        return 0

    @classmethod
    def depthFirstSearch(problem):
        """
        Executes a graph search using the class methods g and h from the class.
        Returns a solution path or None if there is no solution to the problem.
        """
        from util import Stack  # Import necessary classes from within the method

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
                        # Append the current action to the list of actions and update the cost using cls.g
                        newActions = actions + [action]
                        newCost = currentCost + stepCost  # Here we directly use the stepCost, can replace with cls.g logic if needed
                        stack.push((successor, newActions, newCost))

        # Return empty list if no path found
        return []

class BreadthFirstSearch:
    @classmethod
    def g(cls, node):
        """
        Returns the cost to reach the Node node as defined by the search algorithm being implemented.
        In the context of BFS, this can correspond to the depth of the node, as BFS expands nodes level by level.
        """
        return node.depth  # Now using node.depth as BFS typically counts levels

    @classmethod
    def h(cls, node, problem):
        """
        Returns the heuristic cost to the goal as defined by the search algorithm being implemented.
        For BFS, which is an uninformed search algorithm, this is typically 0 as it does not utilize a heuristic.
        """
        return 0  # BFS does not use heuristics

    @classmethod
    def breadthFirstSearch(cls, problem):
        from util import Queue  # Import necessary classes
        # Initialize the queue with the starting point
        start = problem.getStartState()
        queue = Queue()
        queue.push((start, []))  # Each entry is now (Node, actionsToHere)
 # Each entry is now (Node, actionsToHere)

        # Keep track of visited states to avoid cycles
        visited = set()

        while not queue.isEmpty():
            current_node, actions = queue.pop()

            # If this state is the goal, return the actions that got us here
            if problem.isGoalState(current_node.state):
                return actions

            # If the state has not been visited, mark it as visited
            if current_node.state not in visited:
                visited.add(current_node.state)

                # Explore successors
                for successor, action, _ in problem.getSuccessors(current_node.state):
                    # Check if the successor state has already been visited
                    if successor not in visited:
                        # Create a new Node for the successor
                        successor_node = SearchNode(successor, current_node, action)
                        # Add successor to the queue with the corresponding action
                        newActions = actions + [action]
                        queue.push((successor_node, newActions))

        # Return empty list if no path found
        return []


    
class AStarSearch:
    @classmethod
    def g(cls, node):
        """
        Returns the cost to reach the Node node as defined by the search algorithm being implemented.
        For A* search, this is the path cost from the start node to the current node.
        """
        # Assuming 'node' is a tuple where the total path cost to reach the node is the third element
        return node[2]  # node[2] is the currentCost in the provided AStarSearch function

    @classmethod
    def h(cls, node, problem):
        """
        Returns the heuristic cost to the goal as defined by the search algorithm being implemented.
        For A* search, this is an estimate of the cost from the current node to the goal.
        """
        # Here we use the heuristic provided by the problem, but it needs to be defined outside this method.
        return problem.heuristic(node[0], problem)  # node[0] is the currentState

    @classmethod
    def aStarSearch(cls, problem, heuristic):
        """
        Executes an A* search using the class methods g and h from the class.
        Returns a solution path or None if there is no solution to the problem.
        """
        from util import PriorityQueue  # Import necessary classes

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
                        # Calculate new costs using the class methods
                        newCost = cls.g((successor, actions, currentCost + stepCost))
                        totalCost = newCost + cls.h((successor, actions, newCost), problem)
                        # Add successor to the priority queue with the new total cost
                        pq.push((successor, actions + [action], newCost), totalCost)

        # Return empty list if no path found
        return []