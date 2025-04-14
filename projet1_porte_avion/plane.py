"""
Author: Raphael Gravel, Jean-Christophe Chouinard
Date: 15 avril 2025
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
        self.state = PlaneStates.InHangar

    def decolage(self):
        print("\nDécollage en cours...")
        self.state = PlaneStates.Launching
        for i in progressbar(range(10), redirect_stdout=True):
            time.sleep(1)
        self.state = PlaneStates.InAir
        print("L'avion a décollé avec succès!\n")

    def atterissage(self):
        print("\nAtterrissage en cours...")
        self.state = PlaneStates.Landing
        for i in progressbar(range(20), redirect_stdout=True):
            time.sleep(1)
        self.state = PlaneStates.InHangar
        print("L'avion a atterri avec succès!\n")
