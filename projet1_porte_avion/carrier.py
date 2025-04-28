"""
Auteur : Samuel Faucher
Date : 15 avril 2024
Description : Simulation d'un porte-avion commandé par des touches de clavier.
"""

import time
import threading
import multiprocessing
import queue
from plane import Plane, PlaneStates

side = threading.Semaphore(2)
front = threading.Semaphore(2)
runway = threading.Lock()
avions = []
stateLock = threading.Lock()

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
                avion_lance = False
                for avion in avions:
                    with stateLock:
                        if avion.state == PlaneStates.InHangar:
                            threading.Thread(target=avion.decolage, args=(side, front, runway)).start()
                            avion_lance = True
                            break
                if not avion_lance:
                    avions.append(Plane(len(avions)+1, stateLock))
                    threading.Thread(target=avions[len(avions)-1].decolage, args=(side, front, runway)).start()
            elif message == "r":
                with stateLock:
                    for avion in avions:
                        if avion.state == PlaneStates.InAir:
                            threading.Thread(target=avion.atterissage, args=(side, runway)).start()
            elif message == "s":
                for avion in avions:
                    print(f"Avion {avion.id} : {avion.state.name}")
            elif message == "1":
                if(front._value >0):
                    front.acquire()
                    print("Une Catapultes avant Fermé")
                else:
                    print("Les catapultes avant sont déjà tous fermées")
            elif message == "2":
                print("Ouvrir les catapultes avant")
                if(front._value <2):
                    front.release()
                    print("Une Catapultes avant ouvertes")
                else:
                    print("Les catapultes avant sont déjà ouvertes")
            elif message == "3":
                print("Fermer les catapultes Latérales")
                if(side._value >0):
                    side.acquire()
                    print("Une Catapulte latérales fermées")
                else:
                    print("Les catapultes latérales sont déjà fermées")
            elif message == "4":
                print("Ouvrir les catapultes latérales")
                if(side._value <2):
                    side.release()
                    print("Une Catapulte latérales ouvertes")
                else:
                    print("Les catapultes latérales sont déjà ouvertes")
            elif message == "v":
                print("Affichage de l'état des catapultes")
                print(f"Catapultes de coté disponibles:{side._value}")
                print(f"Catapultes d'avant disponible:{front._value}")
                
            elif message == "q":
                with stateLock:
                    for avion in avions:
                        if avion.state == PlaneStates.InAir:
                            threading.Thread(target=avion.atterissage, args=(side, runway)).start()
                flag =True
                while(flag):
                    flag = False
                    with stateLock:
                        for avion in avions:
                            if not avion.state == PlaneStates.InHangar:
                                flag =True
                    time.sleep(0.5)
                                
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
    main()
    pass
