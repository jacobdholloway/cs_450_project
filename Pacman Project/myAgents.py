from pacman import Directions
from game import Agent, Actions
import util

class TimidAgent(Agent):
    def __init__(self):
        super().__init__()

    def inDanger(self, pacman, ghost, dist=3):
        if ghost.scaredTimer > 0: 
            return Directions.STOP
        
        pacmanPos = pacman.getPosition() # Gets the position of Pacman 
        ghostPos = ghost.getPosition() # Gets position of the ghost 

        if pacmanPos[0] == ghostPos[0] and abs(pacmanPos[1] - ghostPos[1]) <= dist: # Based on calculations decides to head N/S/E/W
            return Directions.NORTH if ghostPos[1] > pacmanPos[1] else Directions.SOUTH
        elif pacmanPos[1] == ghostPos[1] and abs(pacmanPos[0] - ghostPos[0]) <= dist:
            return Directions.EAST if ghostPos[0] > pacmanPos[0] else Directions.WEST

        return Directions.STOP

    def getAction(self, state):
        legal = state.getLegalPacmanActions() # Figures out what actions pacman is capable of.
        pacmanState = state.getPacmanState() # Figures out if pacman is scared or not.
        heading = pacmanState.getDirection()

        for ghostState in state.getGhostStates(): 
            dangerDirection = self.inDanger(pacmanState, ghostState)
            if dangerDirection != Directions.STOP: # As long as the ghosts are not in danger it will continue to find and move in directions towards pacman.
                return self.decideOnDanger(dangerDirection, legal, heading)
        
        return self.followLeftTurnBehavior(heading, legal) # Follows left turn behavior to manuever and also uses the heading to move.

    def decideOnDanger(self, dangerDirection, legal, heading):
        reverse = Directions.REVERSE[dangerDirection]
        if reverse in legal and reverse != heading:  # Prefer reversing direction if not currently heading towards danger
            return reverse
        
        # If reversing is not possible or heading towards danger, prioritize turning away from danger
        if dangerDirection in legal:
            oppositeDirection = Directions.REVERSE[heading]
            if oppositeDirection in legal and oppositeDirection != dangerDirection:
                return oppositeDirection
            
        # Try to move left or right from the current heading if directly reversing is not an option
        leftFromDanger = Directions.LEFT[heading]
        if leftFromDanger in legal and leftFromDanger != dangerDirection:
            return leftFromDanger
        rightFromDanger = Directions.RIGHT[heading]
        if rightFromDanger in legal and rightFromDanger != dangerDirection:
            return rightFromDanger
        
        # Last resort: move in the danger direction if it's the only option, otherwise stop
        return Directions.STOP

    def followLeftTurnBehavior(self, heading, legal):
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
