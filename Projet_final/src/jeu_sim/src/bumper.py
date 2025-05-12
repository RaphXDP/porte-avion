#!/usr/bin/env python3
# =============================================================================
# Fichier        : dual_bumper_publisher.py
# Description    : Capteur bumper simulé pour robot_0 et robot_1 basé sur LiDAR
# Auteur         : Jean-Christophe Chouinard
# Date           : 5 mai 2025
# =============================================================================

import rospy                                # Librairie principale de ROS en Python
from sensor_msgs.msg import LaserScan       # Message contenant les données du LiDAR
from std_msgs.msg import Bool               # Message booléen pour l'état du bumper

BUMP_DISTANCE = 0.4  # Seuil de détection (en mètres) : si un obstacle est à moins de 0.4 m

class DualBumperPublisher:
    def __init__(self):
        rospy.init_node("dual_bumper_node")  # Initialise le nœud ROS

        # Création des éditeurs pour chaque robot (publient l'état du bumper)
        self.pub0 = rospy.Publisher("/robot_0/bumper", Bool, queue_size=10)
        self.pub1 = rospy.Publisher("/robot_1/bumper", Bool, queue_size=10)

        # Abonnements aux données LiDAR de chaque robot
        rospy.Subscriber("/robot_0/base_scan", LaserScan, self.lidar_callback_robot0)
        rospy.Subscriber("/robot_1/base_scan", LaserScan, self.lidar_callback_robot1)

        rospy.loginfo("Bumper virtuel initialisé pour robot_0 et robot_1")

    def lidar_callback_robot0(self, msg):
        # Analyse les données du robot_0
        if self._is_bump(msg):
            rospy.loginfo("robot_0 : obstacle détecté !")
        self.pub0.publish(Bool(self._is_bump(msg)))  # Publie True ou False selon la détection

    def lidar_callback_robot1(self, msg):
        # Analyse les données du robot_1
        if self._is_bump(msg):
            rospy.loginfo("robot_1 : obstacle détecté !")
        self.pub1.publish(Bool(self._is_bump(msg)))  # Publie True ou False selon la détection

    def _is_bump(self, scan):
        # Détermine s'il y a un obstacle trop proche devant
        n = len(scan.ranges)
        frontal_ranges = scan.ranges[n//4:3*n//4]  # Prend les lectures centrales (zone frontale)
        # Élimine les valeurs infinies ou nulles
        frontal_ranges = [d for d in frontal_ranges if d > 0 and d < float('inf')]
        if not frontal_ranges:
            return False  # Aucun point valide
        min_distance = min(frontal_ranges)
        return min_distance < BUMP_DISTANCE  # True si obstacle trop proche

    def run(self):
        rospy.spin()  # Boucle d'attente ROS (le nœud tourne en continu)

if __name__ == "__main__":
    try:
        node = DualBumperPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass  # Quitte proprement si ROS interrompt l'exécution
