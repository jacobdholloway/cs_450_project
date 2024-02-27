from pacman import Directions
from game import Agent, Actions
import util

class TimidAgent(Agent):
    def __init__(self):
        super().__init__()

    def inDanger(self, pacman, ghost, dist=3):
        """Check if Pacman is in danger from a ghost.
        
        Args:
            pacman: The state of Pacman (AgentState).
            ghost: The state of a ghost (AgentState).
            dist: Distance threshold to consider danger (default=3).
        
        Returns:
            The direction of danger if Pacman is in danger, otherwise Directions.STOP.
        """
        # Ignore frightened ghosts
        if ghost.scaredTimer > 0:
            return Directions.STOP
        
        pacmanPos = pacman.getPosition()
        ghostPos = ghost.getPosition()

        # Check if in the same row or column and within distance
        if pacmanPos[0] == ghostPos[0] and abs(pacmanPos[1] - ghostPos[1]) <= dist:
            return Directions.NORTH if ghostPos[1] > pacmanPos[1] else Directions.SOUTH
        elif pacmanPos[1] == ghostPos[1] and abs(pacmanPos[0] - ghostPos[0]) <= dist:
            return Directions.EAST if ghostPos[0] > pacmanPos[0] else Directions.WEST

        return Directions.STOP

    def getAction(self, state):
        """Choose an action based on the current state.
        
        Args:
            state: The current game state.
        
        Returns:
            A direction to move in.
        """
        legal = state.getLegalPacmanActions()
        pacmanState = state.getPacmanState()
        heading = pacmanState.getDirection()

        # Check for danger from each ghost
        for ghostState in state.getGhostStates():
            dangerDirection = self.inDanger(pacmanState, ghostState)
            if dangerDirection != Directions.STOP:
                # Decide action based on danger
                return self.decideOnDanger(dangerDirection, legal, heading)
        
        # If not in danger, follow the LeftTurnAgent behavior
        return self.followLeftTurnBehavior(heading, legal)

    def decideOnDanger(self, dangerDirection, legal, heading):
        """Decide on an action when in danger.
        
        Args:
            dangerDirection: The direction of danger.
            legal: List of legal actions.
            heading: Current heading of Pacman.
        
        Returns:
            An action to take.
        """
        reverse = Directions.REVERSE[dangerDirection]
        if reverse in legal:
            return reverse
        leftFromDanger = Directions.LEFT[heading]
        if leftFromDanger in legal:
            return leftFromDanger
        rightFromDanger = Directions.RIGHT[heading]
        if rightFromDanger in legal:
            return rightFromDanger
        return dangerDirection if dangerDirection in legal else Directions.STOP

    def followLeftTurnBehavior(self, heading, legal):
        """Follow the LeftTurnAgent behavior when not in danger.
        
        Args:
            heading: Current heading of Pacman.
            legal: List of legal actions.
        
        Returns:
            A direction to move in.
        """
        if heading == Directions.STOP:
            heading = Directions.NORTH

        left = Directions.LEFT[heading]
        if left in legal:
            return left
        elif heading in legal:
            return heading
        elif Directions.RIGHT[heading] in legal:
            return Directions.RIGHT[heading]
        elif Directions.REVERSE[heading] in legal:
            return Directions.REVERSE[heading]
        return Directions.STOP
