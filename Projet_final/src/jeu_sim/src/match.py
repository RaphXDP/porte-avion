#!/usr/bin/env python3
# =============================================================================
# Fichier        : gestion_match.py
# Description    : Affiche le score actuel du match pour les équipes rouge et bleue.
#                  Met à jour les scores à partir des messages reçus sur le topic "match".
# Auteur         : Jean-Christophe Chouinard
# Date           : 12 mai 2025
# Version        : 1.0
# =============================================================================

import rospy
import json
from std_msgs.msg import String

class GestionMatch:
    def __init__(self):
        rospy.init_node("gestion_match")  # Initialisation du nœud ROS

        # Structure interne pour stocker les scores des deux équipes
        self.score = {
            "bleu": {"ballons": 0, "etages": 0},
            "rouge": {"ballons": 0, "etages": 0}
        }

        self.etat = "En attente"  # État du système (non utilisé activement ici)

        # Souscription au topic "match" pour recevoir les scores
        rospy.Subscriber("match", String, self.mettre_a_jour_score)

        # Timer pour afficher les scores toutes les secondes
        rospy.Timer(rospy.Duration(1), self.affichage_score)

        rospy.loginfo("[Match] Nœud gestion_match actif. Mise à jour directe des scores...")

    def mettre_a_jour_score(self, msg):
        """Met à jour le score reçu sous forme JSON (remplace les valeurs existantes)."""
        try:
            data = json.loads(msg.data)
            for equipe in ["bleu", "rouge"]:
                if equipe in data:
                    self.score[equipe]["ballons"] = data[equipe].get("ballons", 0)
                    self.score[equipe]["etages"] = data[equipe].get("etages", 0)
        except json.JSONDecodeError:
            rospy.logwarn("[Match] Erreur de format JSON dans le message de score.")

    def affichage_score(self, event):
        """Affiche le score total de chaque équipe à la console avec le détail des points."""
        bleu = self.score["bleu"]
        rouge = self.score["rouge"]

        # Calcul des points : 2 points par ballon, 5 points par étage
        total_bleu = bleu["ballons"] * 2 + bleu["etages"] * 5
        total_rouge = rouge["ballons"] * 2 + rouge["etages"] * 5

        print("\n--- Score Actuel ---")
        print(f"Bleu : {total_bleu} pts  (Ballons: {bleu['ballons']} x2, Etages: {bleu['etages']} x5)")
        print(f"Rouge : {total_rouge} pts  (Ballons: {rouge['ballons']} x2, Etages: {rouge['etages']} x5)")
        print(f"→ État du système : {self.etat}")

if __name__ == "__main__":
    try:
        node = GestionMatch()   # Lancement du nœud
        rospy.spin()            # Boucle principale ROS
    except rospy.ROSInterruptException:
        pass  # Ferme proprement en cas d'interruption
