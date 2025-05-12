# Simulation de bumper et gestion de match ROS

Ce projet contient trois nœuds ROS écrits en Python pour simuler des bumpers basés sur un LiDAR et gérer la logique d’un match robotique (chargement, tir et escalade) ainsi que l’affichage des scores.

---

## Table des matières

* [Structure du projet](#structure-du-projet)
* [Prérequis](#prérequis)
* [Installation](#installation)
* [Utilisation](#utilisation)
* [Description des nœuds](#description-des-nœuds)

  * [dual\_bumper\_publisher.py](#dual_bumper_publisherpy)
  * [gestion\_terrain.py](#gestion_terrainpy)
  * [gestion\_match.py](#gestion-matchpy)
* [Topics ROS](#topics-ros)
* [Licence](#licence)

---

## Structure du projet

```
.
├── dual_bumper_publisher.py
├── gestion_terrain.py
├── gestion_match.py
└── README.md
```

---

## Prérequis

* ROS Noetic (ou version compatible Python 3)
* Python 3
* Les paquets ROS :

  * `rospy`
  * `sensor_msgs`
  * `std_msgs`
  * `nav_msgs`

---

## Installation

1. **Cloner le dépôt**

   ```bash
   git clone <url-de-ton-repo>
   cd <nom-du-repo>
   ```

2. **Rendre les fichiers exécutables**

   ```bash
   chmod +x *.py
   ```

3. **Installer les dépendances** (si nécessaire)

   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-std-msgs ros-noetic-nav-msgs
   ```

---

## Utilisation

1. **Lancer roscore**

   ```bash
   roscore
   ```

2. **Lancer chaque nœud dans un nouveau terminal**

   ```bash
   # Terminal 1 : Simulation des bumpers
   rosrun <ton_package> dual_bumper_publisher.py

   # Terminal 2 : Gestion du terrain (chargement, tir, escalade)
   rosrun <ton_package> gestion_terrain.py

   # Terminal 3 : Gestion du match (affichage des scores)
   rosrun <ton_package> gestion_match.py
   ```

---

## Description des nœuds

### dual\_bumper\_publisher.py

* **But** : Simule un capteur “bumper” virtuel pour deux robots (`robot_0` et `robot_1`) à partir de données LiDAR.
* **Fonctionnement** :

  * Souscrit aux topics `/robot_X/base_scan` (type `LaserScan`).
  * Analyse la zone frontale centrale (moitié des mesures), filtre les valeurs invalides, et détecte les obstacles à moins de `0.4 m`.
  * Publie un `Bool` sur `/robot_X/bumper` (`True` si obstacle) et logue l’événement.

### gestion\_terrain.py

* **But** : Gère les actions de jeu des deux équipes (“bleu” et “rouge”) pour trois types d’actions :

  * `charger` (chargement de ballon),
  * `tirer` (lancer de ballon),
  * `escalader` (montée d’étage).
* **Fonctionnement** :

  * Souscrit au topic `robot` (type `String`) pour recevoir des demandes JSON `{ "equipe": ..., "action": ... }`.
  * Lit la position du robot via les topics d’odométrie ROS (`/robot_X/base_pose_ground_truth`).
  * Vérifie que la position est dans la zone autorisée selon l’action et l’équipe.
  * Mécanismes de réussite aléatoire pour `tirer` (90 %) et `escalader` (60 %, max 3 étages).
  * Met à jour le score interne et publie :

    * Sur `terrain` : le résultat de chaque action (`reussite`/`echec`),
    * Sur `match` : le score agrégé sous forme JSON.

### gestion\_match.py

* **But** : Affiche en continu le score global des deux équipes.
* **Fonctionnement** :

  * Souscrit au topic `match` (type `String`) pour recevoir les états de score JSON.
  * Stocke les scores de “ballons” et “étages” pour chaque équipe.
  * À chaque seconde, calcule le score total (`ballons × 2 + étages × 5`) et l’affiche dans la console.

---

## Topics ROS

| Topic                             | Type      | Description                                  |
| --------------------------------- | --------- | -------------------------------------------- |
| `/robot_0/base_scan`              | LaserScan | Entrée LiDAR robot 0                         |
| `/robot_1/base_scan`              | LaserScan | Entrée LiDAR robot 1                         |
| `/robot_0/bumper`                 | Bool      | Sortie bumper virtuel robot 0                |
| `/robot_1/bumper`                 | Bool      | Sortie bumper virtuel robot 1                |
| `robot`                           | String    | Demandes d’actions `{ équipe, action }`      |
| `terrain`                         | String    | Réponses aux actions (résultat JSON)         |
| `match`                           | String    | Score agrégé publié sous forme JSON          |
| `/robot_X/base_pose_ground_truth` | Odometry  | Position réelle des robots (pour la logique) |

---

## Licence

CHAT GPT© 2025 Jean-Christophe Chouinard

---

> **Remarque**
> Pense à ajuster `<ton_package>` lors de l’appel à `rosrun` et à configurer ton `package.xml` et `CMakeLists.txt` si tu intègres ces scripts dans un package ROS complet.
