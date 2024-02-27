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

