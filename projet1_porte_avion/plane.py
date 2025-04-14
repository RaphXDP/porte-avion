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
    def __init__(self, id, stateLock):
        self.state = PlaneStates.InHangar
        self.id = id
        self.stateLock = stateLock
    def decolage(self, side, front, runway):
        with self.stateLock:
            self.state = PlaneStates.WaitingToLaunch
        print(f"\nAvion{self.id} en attente de décollage")

        piste = None

        # Boucle jusqu'à obtenir une piste
        while True:
            if front.acquire(blocking=False):
                piste = 'front'
                break
            elif not runway.locked():
                if side.acquire(blocking=False):
                    piste = 'side'
                    break
            else:
                time.sleep(0.5)  # Attente avant de réessayer

        try:
            print(f"\nAvion{self.id} décollage en cours")
            with self.stateLock:
                self.state = PlaneStates.Launching
            for i in progressbar(range(10), redirect_stdout=True):
                time.sleep(1)
            with self.stateLock:
                self.state = PlaneStates.InAir
            print(f"L'avion{self.id} a décollé avec succès!\n")
        finally:
            if piste == 'front':
                front.release()
            else:
                side.release()

    def atterissage(self, side, runway):
        with self.stateLock:
            self.state = PlaneStates.WaitingToLand
        print(f"\nAvion{self.id} en attente d'aterissage")
        with runway:
            side.acquire()
            side.acquire()
            self.stateLock.acquire()
            self.state = PlaneStates.Landing
            self.stateLock.release()
            for i in progressbar(range(20), redirect_stdout=True):
                time.sleep(1)
            self.stateLock.acquire()
            self.state = PlaneStates.InHangar
            self.stateLock.release()
            print(f"L'avion{self.id} a atterri avec succès!\n")
            side.release()
            side.release()
