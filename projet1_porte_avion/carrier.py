"""
Author: Samuel Faucher
Date: 15 avril 2024
Description: Simulation d un porte-avion commande par des touches de clavier
"""

import time
import threading
import multiprocessing
from ctypes import c_bool

from plane import Plane, PlaneStates

# Fonction qui lit le clavier periodiquement (a partir dans un thread a part)
# Inputs: un booleen qui permet de sortir du thread de facon propre, la queue de message recu
# Output: NA
def dashboard(carrier_active, inputQueue):
    print("Dashboard online")
    while carrier_active.value:
        input_str = input()
        inputQueue.put(input_str)

    print("Dashboard offline")


if __name__ == "__main__":
    pass
