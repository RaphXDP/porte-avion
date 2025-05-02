"""
Fichier : carrier.py
Auteurs : Jean-Christophe Chouinard, Raphaël Gravel
Date : 15 avril 2024

Description :
Simulation d’un porte-avion contrôlé par clavier. Ce programme permet de simuler le lancement 
et l’atterrissage d’avions à l’aide de sémaphores pour modéliser les catapultes avant et latérales.
La communication entre les processus se fait via une file de messages partagée.
"""

import time
import threading
import multiprocessing
import queue
from plane import Plane, PlaneStates

# Sémaphores représentant les catapultes (2 disponibles pour chaque type)
side = threading.Semaphore(2)    # Catapultes latérales
front = threading.Semaphore(2)   # Catapultes avant
runway = threading.Lock()        # Verrou pour la piste unique
avions = []                      # Liste des avions en simulation
stateLock = threading.Lock()     # Verrou pour synchroniser l'accès aux états des avions

def tableau_de_bord(file_entree, drapeau_arret):
    """
    Thread principal du tableau de bord, qui attend les commandes clavier.
    Les commandes sont envoyées via une file à un processus de traitement.
    """
    print("Tableau de bord en ligne")
    while not drapeau_arret.is_set():
        entree = input()
        file_entree.put(entree)
    print("Tableau de bord hors ligne")

def processus_pont(file_messages, drapeau_arret):
    """
    Processus qui reçoit les commandes et agit sur les avions et catapultes.
    Gère les états des avions (décollage, atterrissage), maintenance, etc.
    """
    while not drapeau_arret.is_set():
        try:
            message = file_messages.get(timeout=0.5)

            if message == "STOP":
                drapeau_arret.set()
                break

            elif message == "l":
                # Lance un avion si possible, sinon en crée un nouveau
                avion_lance = False
                for avion in avions:
                    with stateLock:
                        if avion.state == PlaneStates.InHangar:
                            threading.Thread(target=avion.decolage, args=(side, front, runway)).start()
                            avion_lance = True
                            break
                if not avion_lance:
                    avions.append(Plane(len(avions)+1, stateLock))
                    threading.Thread(target=avions[-1].decolage, args=(side, front, runway)).start()

            elif message == "r":
                # Fait atterrir tous les avions en vol
                with stateLock:
                    for avion in avions:
                        if avion.state == PlaneStates.InAir:
                            threading.Thread(target=avion.atterissage, args=(side, runway)).start()

            elif message == "s":
                # Affiche l’état de tous les avions
                for avion in avions:
                    print(f"Avion {avion.id} : {avion.state.name}")

            elif message == "1":
                # Fermer une catapulte avant
                if front._value > 0:
                    front.acquire()
                    print("Une catapulte avant fermée")
                else:
                    print("Toutes les catapultes avant sont déjà fermées")

            elif message == "2":
                # Ouvrir une catapulte avant
                print("Ouvrir les catapultes avant")
                if front._value < 2:
                    front.release()
                    print("Une catapulte avant ouverte")
                else:
                    print("Toutes les catapultes avant sont déjà ouvertes")

            elif message == "3":
                # Fermer une catapulte latérale
                print("Fermer les catapultes latérales")
                if side._value > 0:
                    side.acquire()
                    print("Une catapulte latérale fermée")
                else:
                    print("Toutes les catapultes latérales sont déjà fermées")

            elif message == "4":
                # Ouvrir une catapulte latérale
                print("Ouvrir les catapultes latérales")
                if side._value < 2:
                    side.release()
                    print("Une catapulte latérale ouverte")
                else:
                    print("Toutes les catapultes latérales sont déjà ouvertes")

            elif message == "v":
                # Afficher l’état actuel des catapultes
                print("Affichage de l'état des catapultes")
                print(f"Catapultes de côté disponibles : {side._value}")
                print(f"Catapultes d'avant disponibles : {front._value}")

            elif message == "q":
                # Commencer la procédure de fin de simulation
                with stateLock:
                    for avion in avions:
                        if avion.state == PlaneStates.InAir:
                            threading.Thread(target=avion.atterissage, args=(side, runway)).start()

                # Attendre que tous les avions soient rentrés au hangar
                while True:
                    tous_rentres = True
                    with stateLock:
                        for avion in avions:
                            if avion.state != PlaneStates.InHangar:
                                tous_rentres = False
                                break
                    if tous_rentres:
                        break
                    time.sleep(0.5)

                print("Arrêt du programme — appuyez sur Entrée pour quitter")
                drapeau_arret.set()
                break

            else:
                # Commande inconnue
                print("""\
*******************************************
Touche inconnue pressée. Commandes disponibles :
l = lancer un avion
r = faire atterrir tous les avions
s = afficher l'état de tous les avions
1 = fermer les catapultes avant pour maintenance
2 = rouvrir les catapultes avant
3 = fermer les catapultes latérales pour maintenance
4 = rouvrir les catapultes latérales
v = afficher l'état des catapultes
q + Entrée = accoster le porte-avions (fin du programme)
*******************************************""")

        except queue.Empty:
            continue

def main():
    """
    Point d’entrée du programme.
    Crée le processus de traitement des commandes et démarre l'interface utilisateur.
    """
    file_messages = multiprocessing.Queue()
    manager = multiprocessing.Manager()
    drapeau_arret = manager.Event()

    # Démarre le processus principal de traitement
    processus = multiprocessing.Process(target=processus_pont, args=(file_messages, drapeau_arret))
    processus.start()

    # Lance le tableau de bord (thread principal de saisie)
    while not drapeau_arret.is_set():
        tableau_de_bord(file_messages, drapeau_arret)

    # Attente de la fin du processus
    processus.join()

if __name__ == "__main__":
    main()
