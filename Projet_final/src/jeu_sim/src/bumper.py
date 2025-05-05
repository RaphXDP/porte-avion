#!/usr/bin/env python3
# =============================================================================
# Fichier        : dual_bumper_publisher.py
# Description    : Capteur bumper simulé pour robot_0 et robot_1 basé sur LiDAR
# Auteur         : Jean-Christophe Chouinard
# Date           : 5 mai 2025
# =============================================================================

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

BUMP_DISTANCE = 0.4  # Seuil de détection (en mètres)

class DualBumperPublisher:
    def __init__(self):
        rospy.init_node("dual_bumper_node")
        self.pub0 = rospy.Publisher("/robot_0/bumper", Bool, queue_size=10)
        self.pub1 = rospy.Publisher("/robot_1/bumper", Bool, queue_size=10)

        rospy.Subscriber("/robot_0/base_scan", LaserScan, self.lidar_callback_robot0)
        rospy.Subscriber("/robot_1/base_scan", LaserScan, self.lidar_callback_robot1)

        rospy.loginfo("Bumper virtuel initialisé pour robot_0 et robot_1")

    def lidar_callback_robot0(self, msg):
        if self._is_bump(msg):
            rospy.loginfo("robot_0 : obstacle détecté !")
        self.pub0.publish(Bool(self._is_bump(msg)))

    def lidar_callback_robot1(self, msg):
        if self._is_bump(msg):
            rospy.loginfo("robot_1 : obstacle détecté !")
        self.pub1.publish(Bool(self._is_bump(msg)))

    def _is_bump(self, scan):
        # Analyse la zone frontale centrale
        n = len(scan.ranges)
        frontal_ranges = scan.ranges[n//4:3*n//4]
        # Filtre les valeurs invalides (inf, nan)
        frontal_ranges = [d for d in frontal_ranges if d > 0 and d < float('inf')]
        if not frontal_ranges:
            return False
        min_distance = min(frontal_ranges)
        return min_distance < BUMP_DISTANCE

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = DualBumperPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
