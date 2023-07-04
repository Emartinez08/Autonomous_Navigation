#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class line_follower():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(10)
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.sign_1 = rospy.Subscriber('/traffic/sign_1', String, self.sign_callback_1)
        self.area_1 = rospy.Subscriber("/traffic/sign_area_1", Float32, self.sign_area_cb_1)
        self.sign_2 = rospy.Subscriber('/traffic/sign_2', String, self.sign_callback_2)
        self.area_2 = rospy.Subscriber("/traffic/sign_area_2", Float32, self.sign_area_cb_2)
        twist_msg = Twist()
        #self.robot_vel_pub = rospy.Publisher('/vel/line_follower', Twist, queue_size=1)
        self.robot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.x = 13  # Coordenada x del borde izquierdo
        self.y = 51 # Coordenada y del borde superior
        self.width = 36
        self.height = 13

        self.linear_vel = 0.25  # Velocidad lineal del robot
       
        # Controlador PID
        u_w = [0.0, 0.0]
        e_w = [0.0, 0.0, 0.0]

        kp_w = 0.1
        ki_w = 0.0005
        kd_w = 0.00005

        self.w_max = 1.5
        
        delta_t = 1.0 / 50.0

        left = []
        right = []
        
        left, right = self.read_velocity_data()
        self.image_received_flag = 0

        K1_w = kp_w + delta_t * ki_w + kd_w / delta_t
        K2_w = - kp_w - 2.0 * kd_w / delta_t
        K3_w = kd_w / delta_t
        
        kernel = np.ones((5, 5), np.uint8)
        
        rospy.sleep(2)

        while not rospy.is_shutdown() :
            if self.image_received_flag :
                if (self.sign_1 == 'straight' or self.sign_2 == 'straight') and (self.sign_1 == 'green' or self.sign_2 == 'green'):
                    print("Go forward green")
                    twist_msg.linear.x = 0.25
                    self.robot_vel_pub.publish(twist_msg)

                elif (self.sign_1 == 'straight' or self.sign_2 == 'straight') and (self.sign_1 == 'yellow' or self.sign_2 == 'yellow'):
                    print("Go forward yellow")
                    twist_msg.linear.x = 0.25 / 2.0
                    self.robot_vel_pub.publish(twist_msg)

                elif (self.sign_1 == 'straight' or self.sign_2 == 'straight') and (self.sign_1 == 'red' or self.sign_2 == 'red'):
                    print("Stop red")
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    self.robot_vel_pub.publish(twist_msg)

                elif (self.sign_1 == 'left' or self.sign_2 == 'left' or self.sign_2 == 'right' or self.sign_1 == 'right') and (self.area_1 >= 3700.0 or self.area_2 >=3700.0) and (self.sign_2 == 'green' or self.sign_1 == 'green'):
                    print("Turning green")
                    for linear_vel, right_angular_vel in right:
                        twist_msg = Twist()
                        twist_msg.linear.x = float(linear_vel)
                        twist_msg.angular.z = float(right_angular_vel)
                        self.robot_vel_pub.publish(twist_msg)
                        rate.sleep()

                elif (self.sign_1 == 'left' or self.sign_2 == 'left' or self.sign_2 == 'right' or self.sign_1 == 'right') and (self.area_1 >= 3700.0 or self.area_2 >=3700.0) and (self.sign_2 == 'yellow' or self.sign_1 == 'yellow'):
                    print('Turning yellow')
                    for linear_vel, right_angular_vel in right:
                        twist_msg = Twist()
                        twist_msg.linear.x = float(linear_vel) / 2.0
                        twist_msg.angular.z = float(right_angular_vel) / 2.0
                        self.robot_vel_pub.publish(twist_msg)
                        rate.sleep()

                elif (self.sign_1 == 'left' or self.sign_2 == 'left' or self.sign_2 == 'right' or self.sign_1 == 'right') and (self.area_1 >= 3700.0 or self.area_2 >=3700.0) and (self.sign_2 == 'red' or self.sign_1 == 'red'):
                    print("Turning red")
                    for linear_vel, right_angular_vel in right:
                        twist_msg = Twist()
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        self.robot_vel_pub.publish(twist_msg)
                        rate.sleep()

                elif self.sign_1 == 'works' or self.sign_2 == 'works':
                    print('Works ahead')
                    for linear_vel, right_angular_vel in right:
                        twist_msg = Twist()
                        twist_msg.linear.x = float(linear_vel) / 2.0
                        twist_msg.angular.z = - float(right_angular_vel) / 2.0
                        self.robot_vel_pub.publish(twist_msg)
                        rate.sleep()

                
                elif (self.sign_1 == 'stop' and self.sign_2 == 'stop') and self.area >= 3000.0:
                    print("Stop sign")
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0
                    self.robot_vel_pub.publish(twist_msg)

                else:
                    print('Following line')
                    self.image_received_flag = 0
                    cv_image = self.frame

                    cv_image = cv2.resize(cv_image, (64, 64))

                    roi = cv_image[self.y:self.y+self.height, self.x:self.x+self.width]

                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                    _, binary_roi = cv2.threshold(gray_roi, 85, 255, cv2.THRESH_BINARY_INV)
                    # cv2.imshow("binary", binary_roi)

                    # Buscar los contornos en la imagen binaria
                    contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Si se encuentran contornos
                    if contours :	
                        line_contour = max(contours, key=cv2.contourArea)

                        cv2.drawContours(roi, [line_contour], -1, (0, 0, 255), thickness=2)

                        # Calcular el centro del contorno
                        M = cv2.moments(line_contour)
                        if M['m00'] > 0:
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)

                            # Controlador PID
                            e_w[0] = 19 - cx
                            u_w[0] = K1_w * e_w[0] + K2_w * e_w[1] + K3_w * e_w[2] + u_w[1]
                            e_w[2] = e_w[1]
                            e_w[1] = e_w[0]
                            u_w[1] = u_w[0]

                            if u_w [0] > self.w_max:
                                u_w[0] = self.w_max
                            elif u_w[0] < -self.w_max:
                                u_w[0] = -self.w_max

                            # Control de movimiento
                            angular_z = u_w[0]
                            linear_x = self.linear_vel

                            # Publicar los comandos de velocidad
                            twist_msg.linear.x = linear_x
                            twist_msg.angular.z = angular_z
                            
                            '''if rospy.get_time() - t > 17.4:
                                for linear_vel, right_angular_vel in right:
                                    twist_msg = Twist()
                                    twist_msg.linear.x = float(linear_vel)
                                    twist_msg.angular.z = float(right_angular_vel)
                                    self.robot_vel_pub.publish(twist_msg)
                                    rate.sleep()'''
                            #print(twist_msg)    
                            self.robot_vel_pub.publish(twist_msg)

                # Mostrar la imagen con los contornos y el centro
                # x = cv2.resize(cv_image, (224, 224))
                # cv2.imshow('Camera Viewer', x)

            rate.sleep()

    def image_callback(self, msg):
        try: 
            self.frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") 
            self.image_received_flag = 1 
        except CvBridgeError as e: 
            print(e)

    def sign_callback_1(self, msg_1):
        self.sign_1 = msg_1.data

    def sign_area_cb_1(self, sign_area_1):
        self.sign_area_1 = sign_area_1.data

    def sign_callback_2(self, msg):
        self.sign_2 = msg.data

    def sign_area_cb_2(self, sign_area_2):
        self.sign_area_2 = sign_area_2.data

    def read_velocity_data(self):
        Left = []
        Right = []
        with open('/home/puzzlebot/catkin_ws/src/axolobot/scripts/velocity_data.txt', 'r') as file :
            for line in file :
                linear_vel, angular_vel = line.strip().split(',')
                right_angular_vel = angular_vel
                left_angular_vel = (-1) * angular_vel
                Right.append((linear_vel, right_angular_vel))
                Left.append((linear_vel, left_angular_vel))
        return Left, Right

    def shutdown(self) :
        zero = Twist()
        self.robot_vel_pub.publish(zero)

if __name__ == "__main__":
    rospy.init_node("LINE_FOLLOWER", anonymous=True)
    line_follower()