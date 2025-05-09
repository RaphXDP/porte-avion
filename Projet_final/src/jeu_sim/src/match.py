#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

class GestionMatch:
    def __init__(self):
        rospy.init_node("gestion_match")
        self.score = {
            "bleu": {"ballons": 0, "etages": 0},
            "rouge": {"ballons": 0, "etages": 0}
        }
        self.etat = "En attente"

        rospy.Subscriber("match", String, self.ajouter_score)
        rospy.Timer(rospy.Duration(1), self.affichage_score)

        rospy.loginfo("[Match] Nœud gestion_match actif. Comptabilisation des scores...")

    def ajouter_score(self, msg):
        try:
            data = json.loads(msg.data)
            equipe = data.get("equipe")
            ballons = data.get("ballons", 0)
            etages = data.get("etages", 0)

            if equipe in self.score:
                self.score[equipe]["ballons"] += ballons
                self.score[equipe]["etages"] += etages
            else:
                rospy.logwarn("[Match] Équipe inconnue : %s", equipe)
        except json.JSONDecodeError:
            rospy.logwarn("[Match] Erreur de format JSON dans le message de score.")

    def affichage_score(self, event):
        bleu = self.score["bleu"]
        rouge = self.score["rouge"]
        total_bleu = bleu["ballons"] * 2 + bleu["etages"] * 5
        total_rouge = rouge["ballons"] * 2 + rouge["etages"] * 5

        print("\n--- Score Actuel ---")
        print(f"Bleu : {total_bleu} pts  (Ballons: {bleu['ballons']} x2, Etages: {bleu['etages']} x5)")
        print(f"Rouge : {total_rouge} pts  (Ballons: {rouge['ballons']} x2, Etages: {rouge['etages']} x5)")
        print(f"\u2192 État du système : {self.etat}")

if __name__ == "__main__":
    try:
        node = GestionMatch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
