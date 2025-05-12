#!/usr/bin/env python3
# ============================================================================
# Fichier        : gestion_terrain.py
# Description    : Gère les règles du jeu et les scores des robots sur le terrain.
# Auteur         : Jean-Christophe Chouinard et Raphael gravel 
# Date           : 12 mai 2025
# ============================================================================

import rospy                      # ROS Python API
import json                       # Pour décoder les actions des robots
import random                     # Pour le hasard dans le tir et escalade
import time
import math                       # Pour le calcul de distance dans la zone de tir
from std_msgs.msg import String   # Type de message utilisé pour communication inter-nœuds
from nav_msgs.msg import Odometry  # Pour lire la position du robot
from progressbar import progressbar  # Pour animation console pendant l’action

class GestionTerrain:
    def __init__(self):
        # Initialisation du score par équipe
        self.score = {
            "bleu": {"ballons": 0, "etages": 0},
            "rouge": {"ballons": 0, "etages": 0}
        }

        # Position actuelle de chaque robot
        self.positions = {
            'bleu': {"x": 0, "y": 0},
            'rouge': {"x": 0, "y": 0}
        }

        # Zones définies pour chaque action
        self.zones = {
            "charger": {"bleu": (-6, -2), "rouge": (6, 2)},
            "tirer": {"bleu": (-1, 2), "rouge": (1, -2)},
            "escalader": {"bleu": (-5, 1), "rouge": (5, -1)}
        }

        # Publishers pour envoyer réponses aux robots et scores au match
        self.pub_terrain = rospy.Publisher("terrain", String, queue_size=10)
        self.pub_match = rospy.Publisher("match", String, queue_size=10)

        # Abonnement aux demandes des robots
        rospy.Subscriber("robot", String, self.traiter_demande)

        rospy.loginfo("Node gestion_terrain actif.")

    def publier_score(self):
        # Publie le score actuel des deux équipes au topic "match"
        self.pub_match.publish(json.dumps(self.score))

    def traiter_demande(self, msg):
        # Gère les actions des robots envoyées sous forme JSON
        try:
            demande = json.loads(msg.data)
            equipe = demande["equipe"]
            action = demande["action"]
        except (json.JSONDecodeError, KeyError):
            rospy.logwarn("Demande invalide reçue : %s", msg.data)
            return

        resultat = "echec"  # Valeur par défaut

        # === CHARGER ===
        if action == "charger":
            rospy.loginfo(f"{equipe} demande de charger un ballon...")
            x,y = self.get_position(equipe)
            self.positions[equipe]["x"] = x
            self.positions[equipe]["y"] = y
            if equipe == "bleu":
                if x < self.zones[action][equipe][0] and y < self.zones[action][equipe][1]:
                    resultat = "reussite"
                    rospy.loginfo("chargement réussi!")
                else:
                    rospy.loginfo("chargement échoué!")
            if equipe == "rouge":
                if x > self.zones[action][equipe][0] and y > self.zones[action][equipe][1]:
                    resultat = "reussite"
                    rospy.loginfo("chargement réussi!")
                else:
                    rospy.loginfo("chargement échoué!")

        # === TIRER ===
        elif action == "tirer":
            rospy.loginfo(f"{equipe} tente un tir...")
            x,y = self.get_position(equipe)
            self.positions[equipe]["x"] = x
            self.positions[equipe]["y"] = y
            if self.zone_tir(x, y, equipe):
                for _ in progressbar(range(10), redirect_stdout=True):
                    rospy.sleep(0.1)
                if random.random() <= 0.9:
                    self.score[equipe]["ballons"] += 1
                    resultat = "reussite"
                    rospy.loginfo("Tir réussi!")
                else:
                    rospy.loginfo("Tir échoué.")
            else:
                rospy.loginfo("Robot pas au bon endroit pour tirer")
            self.publier_score()

        # === ESCALADER ===
        elif action == "escalader":
            rospy.loginfo(f"{equipe} tente de grimper...")
            x,y = self.get_position(equipe)
            self.positions[equipe]["x"] = x
            self.positions[equipe]["y"] = y
            if equipe == "bleu":
                if x < self.zones[action][equipe][0] and y > self.zones[action][equipe][1] and self.score[equipe]["etages"] < 3:
                    if random.random() <= 0.6:
                        self.score[equipe]["etages"] += 1
                        resultat = "reussite"
                        rospy.loginfo("Escalade réussie!")
                    else:
                        rospy.loginfo("Échec de l'escalade.")
            if equipe == "rouge":
                if x > self.zones[action][equipe][0] and y < self.zones[action][equipe][1] and self.score[equipe]["etages"] < 3:
                    if random.random() <= 0.6:
                        self.score[equipe]["etages"] += 1
                        resultat = "reussite"
                        rospy.loginfo("Escalade réussie!")
                    else:
                        rospy.loginfo("Échec de l'escalade.")
            self.publier_score()

        # Envoi de la réponse à l'action
        reponse = {
            "equipe": equipe,
            "action": action,
            "resultat": resultat
        }
        self.pub_terrain.publish(json.dumps(reponse))

    def get_position(self, equipe):
        # Va chercher la position actuelle du robot dans Gazebo via Odometry
        topic = "/robot_0/base_pose_ground_truth" if equipe == "bleu" else "/robot_1/base_pose_ground_truth"
        try:
            msg = rospy.wait_for_message(topic, Odometry, timeout=1.0)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            return (x, y)
        except rospy.ROSException:
            rospy.logwarn(f"Position non disponible pour l’équipe {equipe}.")
            return (0, 0)  # Valeur par défaut si position manquante

    def zone_tir(self, x, y, equipe):
        # Vérifie si le robot est dans la zone de tir autorisée
        distance = math.sqrt(x**2 + y**2)
        rayon = math.sqrt((self.zones["tirer"]["rouge"][0] - self.zones["tirer"]["bleu"][0])**2 + (self.zones["tirer"]["rouge"][1] - self.zones["tirer"]["bleu"][1])**2)/2
        val = -4 * x - 2 * y

        if distance > rayon:
            rospy.loginfo("Robot pas dans le cercle de tir")
            return False

        if equipe == "bleu" and val > 0:
            return True
        elif equipe == "rouge" and val < 0:
            return True

        rospy.loginfo("Robot pas dans la zone de tir")
        return False

if __name__ == "__main__":
    try:
        rospy.init_node("gestion_terrain")
        GestionTerrain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
