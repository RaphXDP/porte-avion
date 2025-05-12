#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
import threading
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from progressbar import progressbar

class Robot:
    def __init__(self, name, cmd_topic, team_color, keys):
        self.name = name
        self.team_color = team_color
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        self.cmd = Twist()
        self.ball_count = 0
        self.max_balls = 2
        self.climbing = False
        self.loading = False
        self.keys = keys
        self.confirmation_result = None
        self.successful_climb = False
        self.current_key = None
        self.running = True
        self.bumped = False
        self.bump_escape = False
        self.newresult = False
        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.start()

        rospy.Subscriber(f"/{self.name}/bumper", Bool, self.bumper_callback)

    def bumper_callback(self, msg):
        was_bumped = self.bumped
        self.bumped = msg.data
        if self.bumped and not was_bumped:
            rospy.loginfo(f"{self.name} : collision détectée ! Tentative de recul.")
            self.bump_escape = True
        elif not self.bumped and was_bumped:
            rospy.loginfo(f"{self.name} : obstacle dégagé. Reprise normale.")
            self.bump_escape = False

    def handle_key(self, key):
        if self.climbing:
            self.stop()
            if key == self.keys['climb']:
                self.climb()
            else:
                rospy.loginfo(f"{self.name} escalade : bloqué.")
            return
        
        elif self.loading:
            self.stop()
            rospy.loginfo(f"{self.name} rechargement : bloqué.")
            return


        elif key in self.keys['move']:
            if self.current_key == key:
                self.current_key = None
                self.stop()
                rospy.loginfo(f"{self.name} : arrêt du mouvement.")
            else:
                self.current_key = key
                rospy.loginfo(f"{self.name} : mouvement actif -> {key}")
        elif key == self.keys['charge']:
            self.stop()
            self.current_key = key
            self.load_ball()
        elif key == self.keys['tir']:
            self.stop()
            self.current_key = key
            self.shoot_ball()
        elif key == self.keys['climb']:
            self.stop()
            self.current_key = key
            self.climb()

    def movement_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.running:
            if self.bump_escape:
                self.cmd.linear.x = -0.3
                self.cmd.angular.z = 0.0
                self.cmd_pub.publish(self.cmd)
            elif self.current_key:
                self.move(self.current_key)
            else:
                self.stop()
            rate.sleep()

    def move(self, key):
        directions = {
            'w': (0.5, 0), 's': (-0.5, 0), 'a': (0, 0.5), 'd': (0, -0.5),
            'i': (0.5, 0), 'k': (-0.5, 0), 'j': (0, 0.5), 'l': (0, -0.5)
        }
        lin, ang = directions.get(key, (0, 0))
        self.cmd.linear.x = lin
        self.cmd.angular.z = ang
        self.cmd_pub.publish(self.cmd)

    def stop(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.cmd_pub.publish(self.cmd)

    def load_ball(self):
        if self.ball_count >= self.max_balls:
            rospy.loginfo(f"{self.name} a déjà {self.max_balls} ballons.")
            return
        self.newresult = False
        self.send_action('charger')
        self.loading = True
        rospy.loginfo(f"{self.name} attend un ballon...")
        while not self.newresult:
           rospy.loginfo(f"{self.newresult}")
           rospy.sleep(0.1)
        for _ in progressbar(range(10), redirect_stdout=True):
                rospy.sleep(0.1)
        self.loading = False
        if self.confirmation_result == 'reussite':
            self.ball_count += 1
            rospy.loginfo(f"{self.name} a maintenant {self.ball_count} ballons.")
        else:
            rospy.loginfo(f"{self.name} : échec de chargement.")

    def shoot_ball(self):
        if self.ball_count == 0:
            rospy.loginfo(f"{self.name} n'a pas de ballon à tirer.")
            return
        self.newresult = False
        self.send_action('tirer')
        
        rospy.loginfo(f"{self.name} tente un tir...")
        while not self.newresult:
            rospy.sleep(0.1)
        if self.confirmation_result == 'reussite':
            self.ball_count -= 1
            rospy.loginfo(f"{self.name} a marqué 2 points. Ballons restants : {self.ball_count}")
        else:
            rospy.loginfo(f"{self.name} : tir échoué.")

    def climb(self):
        
        self.newresult = False
        self.send_action('escalader')
        rospy.loginfo(f"{self.name} tente l’escalade...")
        while not self.newresult:
            rospy.sleep(0.1)
        for _ in progressbar(range(10), redirect_stdout=True):
                rospy.sleep(0.3)
        if self.confirmation_result == 'reussite':
            self.climbing = True
            rospy.loginfo(f"{self.name} a réussi l’escalade (+5 points)")
        else:
            rospy.loginfo(f"{self.name} a échoué l’escalade.")

    def send_action(self, action):
        msg = String()
        data = {"equipe": self.team_color, "action": action}
        msg.data = json.dumps(data)
        robot_pub.publish(msg)

    def update_confirmation(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("equipe") == self.team_color and data.get("action") in ["tirer", "charger", "escalader"]:
                self.confirmation_result = data.get("resultat")
                self.newresult = True
                rospy.loginfo(f"{self.newresult} callback")
        except Exception as e:
            rospy.logwarn(f"{self.name} a reçu une donnée non valide : {e}")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    rospy.init_node('robot_node')

    robot_pub = rospy.Publisher('/robot', String, queue_size=10)

    robot1_keys = {'move': ['w', 'a', 's', 'd'], 'charge': 'z', 'tir': 'x', 'climb': 'c'}
    robot2_keys = {'move': ['i', 'j', 'k', 'l'], 'charge': 'b', 'tir': 'n', 'climb': 'm'}

    robot1 = Robot("robot_0", "/robot_0/cmd_vel", "bleu", robot1_keys)
    robot2 = Robot("robot_1", "/robot_1/cmd_vel", "rouge", robot2_keys)

    rospy.Subscriber("/terrain", String, robot1.update_confirmation)
    rospy.Subscriber("/terrain", String, robot2.update_confirmation)

    rospy.loginfo("Contrôle actif : touches robot1 = wasd/zxc, robot2 = ijkl/bnm. Appuie sur 'q' pour quitter.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'q':
                robot1.running = False
                robot2.running = False
                robot1.movement_thread.join()
                robot2.movement_thread.join()
                rospy.loginfo("Fermeture demandée.")
                break
            robot1.handle_key(key)
            robot2.handle_key(key)
    except rospy.ROSInterruptException:
        pass