#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

rospy.init_node('line_follower')

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

cv_bridge = CvBridge()

def image_callback(msg):
    try:
        img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_filtree = color_filter(img_rgb)
        img_roi = roi(img_filtree)

        controle_suivi_ligne(img_roi)
    except CvBridgeError as e:
        rospy.logerr(e)

# Filtrer l'image pour ne conserver que les pixels rouges
def color_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    lower_red = np.array([0, 100, 100])  # Plage de valeurs pour le rouge (vous pouvez ajuster ces valeurs si nécessaire)
    upper_red = np.array([10, 255, 255])  # Plage de valeurs pour le rouge
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    masked_image = np.zeros_like(image)
    masked_image[red_mask != 0] = [255, 255, 255]  # Remplacez le rouge par du blanc
    return masked_image

def roi(im):
    x = int(im.shape[1])
    y = int(im.shape[0])

    # Définir les coordonnées de la région d'intérêt en pourcentage de la largeur et de la hauteur de l'image
    shape = np.array([[int(0.003 * x), int(0.7651 * y)],
                      [int(0.995 * x), int(0.735 * y)],
                      [int(0.552 * x), int(0.514 * y)],
                      [int(0.445 * x), int(0.52 * y)]])

    mask = np.zeros_like(im)

    if len(im.shape) > 2:
        channel_count = im.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)
    masked_image = cv2.bitwise_and(im, mask)
    return masked_image

# Contrôler le robot en fonction de l'image de la région d'intérêt
def controle_suivi_ligne(image_roi):
   
   # touuuuuuuuut le code pour que ça suit la ligne 

    # Exemple de commande de vitesse :
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.2  
    cmd_vel.angular.z = 0.0  # Pas de rotation (rad/s)
    cmd_vel_pub.publish(cmd_vel)

rospy.Subscriber('/camera/image', Image, image_callback)
rospy.spin()
