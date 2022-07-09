#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  5 21:28:18 2022

@author: mcunbagy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import imutils


class TerrainNode(Node):
    
    def __init__(self):
        super().__init__('map')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.on_image_received,
            1)
        self.subscription  # prevent unused variable warning

    def on_image_received(self, msg):
        img = np.array(msg.data).reshape((240, 320,3))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # yellow color 
        lower_yellow = np.array([25,100,100])
        upper_yellow = np.array([30,255,255])
        #blue color 
        lower_blue = np.array([90,60,0])
        upper_blue = np.array([121,255,255])
        #green color 
        lower_green = np.array([40,70,80])
        upper_green = np.array([70,255,255])

        yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green = cv2.inRange(hsv, lower_green, upper_green)
        blue = cv2.inRange(hsv, lower_blue, upper_blue)

        cnts1 = cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts1 = imutils.grab_contours(cnts1)
        
        cnts2 = cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = imutils.grab_contours(cnts2)
        
        cnts3 = cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts3 = imutils.grab_contours(cnts3)
        

        
        center_list = []
        colored_contours = [cnts1,cnts2,cnts3]
       
        #total of 6 elements indicating colored contours, lets find centers and draw.
        for i in colored_contours:
            for c in i:
                cv2.drawContours(img,[c],-1,(0,255,0),3)
                # compute the center of the contour
                M = cv2.moments(c)
                if M["m00"] != 0:
                 cX = int(M["m10"] / M["m00"])
                 cY = int(M["m01"] / M["m00"])
                 center_list.append([cX,cY])
                else:
                 cX, cY = 0, 0
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), 1)
                cv2.putText(img, "red", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 1)
                

        cv2.imshow('frame',img)
        k = cv2.waitKey(100)
        if k == 27:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = TerrainNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
