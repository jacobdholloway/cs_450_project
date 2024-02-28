# Programming assignment Affadit

# Single programmer:
# I promise that the attached assignment is my own work. I recognize that should this not
# be the case, I will be subject to penalties as outlined in the course syllabus. Jacob Holloway


# • Multiple programmer:
# We the undersigned promise that we have in good faith attempted to follow the principles
# of pair programming. Although we were free to discuss ideas with others, the
# implementation is our own. We have shared a common workspace (possibly virtually) and
# taken turns at the keyboard for the majority of the work that we are submitting.
# Furthermore, any non programming portions of the assignment were done independently.
# We recognize that should this not be the case, we will be subject to penalties as outlined
# in the course syllabus. [Your Names]


# Created by Jacob Holloway
# Made for CS 450 Intro to AI
# explore.py is part of Assignment 2 part 2
# Worked on search.py and explore.py


class Explored(object):
    def __init__(self):
        self.hashtable = set()  # Use a set to store explored states for efficient lookup

    def exists(self, state):
        return state in self.hashtable

    def add(self, state):
        self.hashtable.add(state)

