#!/usr/bin/env python3
import rospy
import json
import random
import time
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class GestionTerrain:
    def __init__(self):
        self.score = {
            "bleu": {"ballons": 0, "etages": 0},
            "rouge": {"ballons": 0, "etages": 0}
        }

        self.positions = {
            "bleu": {"x": 0, "y": 0},
            "rouge": {"x": 0, "y": 0}
        }

        self.zones = {
        "charger": {"bleu": (-6, -2), "rouge": (6, 2)},
        "tirer": {"bleu": (-1, 2), "rouge": (1, -2)},
        "escalader": {"bleu": (-5, 1), "rouge": (5, -1)}
        }

        self.pub_terrain = rospy.Publisher("terrain", String, queue_size=10)
        self.pub_match = rospy.Publisher("match", String, queue_size=10)

        rospy.Subscriber("robot", String, self.traiter_demande)
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, self.maj_position_bleu)
        rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, self.maj_position_rouge)

        rospy.loginfo("Node gestion_terrain actif.")

    def publier_score(self):

        self.pub_match.publish(json.dumps(self.score))

    def traiter_demande(self, msg):
        try:
            demande = json.loads(msg.data)
            equipe = demande["equipe"]
            action = demande["action"]
        except (json.JSONDecodeError, KeyError):
            rospy.logwarn("Demande invalide reçue : %s", msg.data)
            return

        resultat = "echec"

        if action == "charger":
            rospy.loginfo(f"{equipe} demande de charger un ballon...")
            self.positions[equipe] = self.get_position(equipe)
            if equipe == "bleu":
                if self.positions[equipe]["x"] < self.zones[action][equipe][0] and self.positions[equipe]["y"] < self.zones[action][equipe][1]:
                    resultat = "reussite"
            if equipe == "rouge":
                if self.positions[equipe]["x"] > self.zones[action][equipe][0] and self.positions[equipe]["y"] > self.zones[action][equipe][1]:
                    resultat = "reussite"
            

        elif action == "tirer":
            rospy.loginfo(f"{equipe} tente un tir...")
            if(self.zone_tir(self.positions[equipe][0], self.positions[equipe][1], equipe)):
                if random.random(1, 10) < 9:
                    self.score[equipe]["ballons"] += 1
                    resultat = "reussite"
                    rospy.loginfo("Tir réussi!")
                else:
                    rospy.loginfo("Tir échoué.")
                self.publier_score()

        elif action == "escalader":
            rospy.loginfo(f"{equipe} tente de grimper...")
            if equipe == "bleu":
                if self.positions[equipe]["x"] < self.zones[action][equipe][0] and self.positions[equipe]["y"] > self.zones[action][equipe][1]:
                    if random.random(1, 10) < 6:
                        self.score[equipe]["etages"] += 1
                        resultat = "reussite"
                        rospy.loginfo("Escalade réussie!")
                    else:
                        rospy.loginfo("Échec de l'escalade.")
            if equipe == "rouge":
                if self.positions[equipe]["x"] > self.zones[action][equipe][0] and self.positions[equipe]["y"] < self.zones[action][equipe][1]:
                    if random.random(1, 10) < 6:
                        self.score[equipe]["etages"] += 1
                        resultat = "reussite"
                        rospy.loginfo("Escalade réussie!")
                    else:
                        rospy.loginfo("Échec de l'escalade.")
            self.publier_score()

        reponse = {
            "equipe": equipe,
            "action": action,
            "resultat": resultat
        }
        self.pub_terrain.publish(json.dumps(reponse))

    def get_position(self, equipe):
        topic = "/robot_0/base_pose_ground_truth" if equipe == "bleu" else "/robot_1/base_pose_ground_truth"
        try:
            msg = rospy.wait_for_message(topic, Odometry, timeout=1.0)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            return (x, y)
        except rospy.ROSException:
            rospy.logwarn(f"Position non disponible pour l’équipe {equipe}.")
            return None
    def zone_tir(self, x, y, equipe):

        distance = math.sqrt(x**2 + y**2)
        rayon = math.sqrt((self.zones["rouge"][0] - self.zones["bleu"][0])**2 + (self.zones["rouge"][1] - self.zones["bleu"][1])**2)/2

        val = -4 * x - 2 * y

        if distance > rayon:
            return False

        if equipe == "bleu" and val > 0:
            return True
        elif equipe == "rouge" and val < 0:
            return True

        return False

if __name__ == "__main__":
    try:
        rospy.init_node("gestion_terrain")
        GestionTerrain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
