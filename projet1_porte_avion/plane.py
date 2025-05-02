"""
Fichier : plane.py
Auteurs : Raphaël Gravel, Jean-Christophe Chouinard
Date : 15 avril 2025

Description :
Ce module modélise les différentes phases d’un avion opérant sur un porte-avions.
Il définit les états d’un avion, et implémente les fonctions de décollage et d’atterrissage,
en utilisant des verrous pour synchroniser les accès aux ressources partagées comme les catapultes.
"""

import time
import random
from enum import Enum
from progressbar import progressbar

# Énumération représentant les états possibles d’un avion
class PlaneStates(Enum):
    InHangar = 1         # Dans le hangar
    WaitingToLaunch = 2  # En attente de décollage
    Launching = 3        # En cours de décollage
    InAir = 4            # En vol
    WaitingToLand = 5    # En attente d’atterrissage
    Landing = 6          # En cours d’atterrissage
    Retired = 7          # Retiré du service (non utilisé ici)

# Classe représentant un avion
class Plane:
    def __init__(self, id, stateLock):
        self.state = PlaneStates.InHangar    # État initial dans le hangar
        self.id = id                         # Identifiant unique de l’avion
        self.stateLock = stateLock           # Verrou partagé pour synchroniser les états

    def decolage(self, side, front, runway):
        """
        Fonction qui simule le décollage de l’avion. Elle tente d’acquérir une catapulte disponible.
        Priorité est donnée aux catapultes avant, puis aux latérales si la piste est libre.
        """
        with self.stateLock:
            self.state = PlaneStates.WaitingToLaunch
        print(f"\nAvion{self.id} en attente de décollage")

        piste = None

        # Essayer d’obtenir une catapulte disponible
        while True:
            if front.acquire(blocking=False):
                piste = 'front'  # Catapulte avant obtenue
                break
            elif not runway.locked():
                if side.acquire(blocking=False):
                    piste = 'side'  # Catapulte latérale obtenue
                    break
            else:
                time.sleep(0.5)  # Attendre avant de réessayer

        try:
            print(f"\nAvion{self.id} décollage en cours")
            with self.stateLock:
                self.state = PlaneStates.Launching
            for _ in progressbar(range(1), redirect_stdout=True):
                time.sleep(1)
            with self.stateLock:
                self.state = PlaneStates.InAir
            print(f"L'avion{self.id} a décollé avec succès!\n")
        finally:
            # Libère la catapulte utilisée
            if piste == 'front':
                front.release()
            else:
                side.release()

    def atterissage(self, side, runway):
        """
        Fonction qui simule l’atterrissage de l’avion.
        Nécessite la piste ainsi que deux catapultes latérales.
        """
        with self.stateLock:
            self.state = PlaneStates.WaitingToLand
        print(f"\nAvion{self.id} en attente d'atterrissage")

        with runway:  # Acquérir la piste
            side.acquire()
            side.acquire()  # Deux catapultes latérales nécessaires

            # Passage à l’état d’atterrissage
            self.stateLock.acquire()
            self.state = PlaneStates.Landing
            self.stateLock.release()

            for _ in progressbar(range(2), redirect_stdout=True):
                time.sleep(1)

            # Retour au hangar
            self.stateLock.acquire()
            self.state = PlaneStates.InHangar
            self.stateLock.release()

            print(f"L'avion{self.id} a atterri avec succès!\n")

            side.release()
            side.release()
