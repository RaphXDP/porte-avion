"""
Auteur : Samuel Faucher
Date : 15 avril 2024
Description : Simulation d'un porte-avion commandé par des touches de clavier.
"""

import time
import threading
import multiprocessing
from ctypes import c_bool
import queue
from plane import Plane, PlaneStates

def tableau_de_bord(file_entree, drapeau_arret):
    """
    Lit le clavier périodiquement et envoie les entrées dans une file.
    Args:
        file_entree (multiprocessing.Queue): File pour les messages.
        drapeau_arret (multiprocessing.Event): Indique si le programme doit s'arrêter.
    """
    print("Tableau de bord en ligne")
    while not drapeau_arret.is_set():
        entree = input()
        file_entree.put(entree)
    print("Tableau de bord hors ligne")

def processus_pont(file_messages, drapeau_arret):
    """
    Traite les messages reçus et affiche les actions correspondantes.
    Args:
        file_messages (multiprocessing.Queue): File contenant les commandes clavier.
        drapeau_arret (multiprocessing.Event): Indique si le programme doit s'arrêter.
    """
    while not drapeau_arret.is_set():
        try:
            message = file_messages.get(timeout=0.5)

            if message == "STOP":
                drapeau_arret.set()
                break
            elif message == "l":
                print("Lancer un avion")
            elif message == "r":
                print("Rapatrier tous les avions")
            elif message == "s":
                print("Affichage des états des avions")
            elif message == "1":
                print("Fermer les catapultes avant pour maintenance")
            elif message == "2":
                print("Ouvrir les catapultes avant")
            elif message == "3":
                print("Fermer les catapultes latérales pour maintenance")
            elif message == "4":
                print("Ouvrir les catapultes latérales")
            elif message == "v":
                print("Affichage de l'état des catapultes")
            elif message == "q":
                print("Arrêt du programme — appuyez sur Entrée pour quitter")
                drapeau_arret.set()
                break
            else:
                print("""\
*******************************************
Touche inutile pressée. Les touches utiles sont :
l = lancer un avion
r = faire atterrir tous les avions
s = afficher l'état de tous les avions
1 = fermer les catapultes avant pour maintenance
2 = rouvrir les catapultes avant
3 = fermer les catapultes latérales pour maintenance
4 = rouvrir les catapultes latérales
v = afficher l'état des catapultes
q + _ = accoster le porte-avions (fin du programme)
*******************************************""")

            print(message)

        except queue.Empty:
            continue

def main():
    file_messages = multiprocessing.Queue()
    manager = multiprocessing.Manager()
    drapeau_arret = manager.Event()

    processus = multiprocessing.Process(target=processus_pont, args=(file_messages, drapeau_arret))
    processus.start()

    while not drapeau_arret.is_set():
        tableau_de_bord(file_messages, drapeau_arret)

    processus.join()

if __name__ == "__main__":
<<<<<<< HEAD
    main()
=======
    avion = Plane()
    avion.decolage()
    time.sleep(2)
    avion.atterissage()
    pass
>>>>>>> 71209950eb4fc5e17ca7b51a063eea48bd735086
