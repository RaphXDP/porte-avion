"""
Author: Samuel Faucher
Date: 15 avril 2024
Description: Simulation des differentes phases d une avion sur un porte-avion
"""

import time
import random
from enum import Enum
from progressbar import progressbar

# Etats possibles d un avion
class PlaneStates(Enum):
    InHangar = 1
    WaitingToLaunch = 2
    Launching = 3
    InAir = 4
    WaitingToLand = 5
    Landing = 6
    Retired = 7

# Objet regroupant les fonctionnalites d un avion
class Plane:
    def __init__(self):
        pass

    def fonction(self):
        pass


