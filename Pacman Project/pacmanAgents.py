# pacmanAgents.py
# ---------------
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


from pacman import Directions
from game import Agent
import random
import game
import util

class LeftTurnAgent(game.Agent):
    "An agent that turns left at every opportunity"

    def getAction(self, state):
        """getAction(state) - Make a decsion based on the current game state
        state - pacman.GameState instance
        returns a valid action direction:  North, East, South, West or
        a Stop action when no legal actions are possible
        """

        # List of directions the agent can choose from
        legal = state.getLegalPacmanActions()

        # Get the agent's state from the game state and find agent heading
        agentState = state.getPacmanState()
        heading = agentState.getDirection()

        if heading == Directions.STOP:
            # Pacman is stopped, assume North (true at beginning of game)
            heading = Directions.NORTH

        # Turn left if possible
        left = Directions.LEFT[heading]  # What is left based on current heading
        if left in legal:
            action = left
        else:
            # No left turn
            if heading in legal:
                action = heading  # continue in current direction
            elif Directions.RIGHT[heading] in legal:
                action = Directions.RIGHT[heading]  # Turn right
            elif Directions.REVERSE[heading] in legal:
                action = Directions.REVERSE[heading]  # Turn around
            else:
                action = Directions.STOP  # Can't move!

        return action

class GreedyAgent(Agent):
    def __init__(self, evalFn="scoreEvaluation"):
        self.evaluationFunction = util.lookup(evalFn, globals())
        assert self.evaluationFunction != None

    def getAction(self, state):
        # Generate candidate actions
        legal = state.getLegalPacmanActions()
        if Directions.STOP in legal: legal.remove(Directions.STOP)

        successors = [(state.generateSuccessor(0, action), action) for action in legal]
        scored = [(self.evaluationFunction(state), action) for state, action in successors]
        bestScore = max(scored)[0]
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        return random.choice(bestActions)
    
class HybridAgent(Agent):
    def __init__(self, evalFn="scoreEvaluation"):
        self.evaluationFunction = util.lookup(evalFn, globals())
        assert self.evaluationFunction != None

    def inDanger(self, pacman, ghost, dist=3):
        # Immediately return STOP if the ghost is frightened, indicating no danger from this ghost
        if ghost.scaredTimer > 0:
            return Directions.STOP

        pacmanPos = pacman.getPosition()
        ghostPos = ghost.getPosition()

        # Check if in the same row or column and within distance
        if pacmanPos[0] == ghostPos[0]:  # Same column
            if abs(pacmanPos[1] - ghostPos[1]) <= dist:
                return Directions.NORTH if ghostPos[1] > pacmanPos[1] else Directions.SOUTH
        elif pacmanPos[1] == ghostPos[1]:  # Same row
            if abs(pacmanPos[0] - ghostPos[0]) <= dist:
                return Directions.EAST if ghostPos[0] > pacmanPos[0] else Directions.WEST

        return Directions.STOP  # Not in danger or ghost is frightened

    def getAction(self, state):
        legal = state.getLegalPacmanActions()
        if Directions.STOP in legal: legal.remove(Directions.STOP)

        agentState = state.getPacmanState()

        # No need to check for danger from frightened ghosts, as inDanger handles it
        for ghostState in state.getGhostStates():
            if self.inDanger(agentState, ghostState) != Directions.STOP:
                successors = [(state.generateSuccessor(0, action), action) for action in legal]
                scored = [(self.evaluationFunction(state), action) for state, action in successors]
                bestScore = max(scored, key=lambda x: x[0])[0]
                bestActions = [action for (score, action) in scored if score == bestScore]
                return random.choice(bestActions)

        # Attempt to turn left if possible
        heading = agentState.getDirection()
        left = Directions.LEFT[heading]
        if left in legal:
            return left
        
        # If left is not an option, proceed with the greedy approach
        successors = [(state.generateSuccessor(0, action), action) for action in legal]
        scored = [(self.evaluationFunction(state), action) for state, action in successors]
        bestScore = max(scored, key=lambda x: x[0])[0]
        bestActions = [action for (score, action) in scored if score == bestScore]

        return random.choice(bestActions)



def scoreEvaluation(state):
    return state.getScore()
