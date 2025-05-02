# Simulation Porte-Avions

## Auteurs
- Jean-Christophe Chouinard  
- Raphaël Gravel

## Description

Ce projet simule le fonctionnement d’un porte-avions à l’aide de processus et de threads Python. Il permet de :
- Lancer et faire atterrir des avions.
- Gérer des catapultes avant et latérales à l’aide de sémaphores.
- Suivre l’état de chaque avion.
- Contrôler le tout via des commandes clavier dans une interface console.

L’objectif principal est de simuler de manière réaliste les contraintes liées aux ressources limitées d’un porte-avions (piste unique, nombre limité de catapultes, coordination des avions).

---

## Structure du projet

- **`porte_avion.py`**  
  Point d’entrée du programme. Gère les interactions utilisateur, le traitement des commandes, et le lancement/atterrissage des avions via des processus et threads.

- **`plane.py`**  
  Contient la classe `Plane` et l’énumération `PlaneStates`. Chaque avion suit un cycle d’états : hangar → lancement → vol → atterrissage → hangar.

---

## Commandes disponibles (à entrer dans la console pendant l'exécution)

| Touche | Action |
|--------|--------|
| `l`    | Lancer un avion |
| `r`    | Faire atterrir tous les avions en vol |
| `s`    | Afficher l’état de tous les avions |
| `1`    | Fermer une catapulte avant (maintenance) |
| `2`    | Ouvrir une catapulte avant |
| `3`    | Fermer une catapulte latérale (maintenance) |
| `4`    | Ouvrir une catapulte latérale |
| `v`    | Afficher l’état des catapultes disponibles |
| `q`    | Faire atterrir tous les avions puis quitter le programme |
| `Entrée` après `q` | Confirmer la sortie |

---

## Dépendances

Installe les bibliothèques nécessaires avec :

```bash
pip install progressbar2
