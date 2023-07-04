#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Logic_Controller() :
    def __init__(self) :
        rospy.on_shutdown(self.shutdown)

        node_rate = 10
        rate = rospy.Rate(node_rate)

        rospy.Subscriber("/traffic/sign_name", Int8, self.sign_name_cb)
        rospy.Subscriber("/traffic/light_color", String, self.light_color_cb)
        rospy.Subscriber("/vel/behaviour_on_sign", Twist, self.behaviour_on_sign_vel_cb)
        rospy.Subscriber("/traffic/sign_area", Float32, self.sign_area_cb)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.behaviour_on_sign_vel = Twist()
        self.zero = Twist()
        self.vel = Twist()

        self.sign_area = 0.0
        self.sign_name = ""
        self.light_color = ""
        self.sign_area = 0.0
        self.sign_on = 0
        
        left = []
        right = []
        
        left, right = self.read_velocity_data()

        while not rospy.is_shutdown():
            if self.sign_area >= 3000.0:
                if self.light_color == "red":
                    print("Seeing red")
                    self.vel.linear.x = self.zero.linear.x
                    self.vel.angular.z = self.zero.angular.z
                elif self.light_color == "yellow":
                    print("Seeing yellow")
                    self.vel.linear.x = self.behaviour_on_sign_vel.linear.x / 2.0
                    self.vel.angular.z = self.behaviour_on_sign_vel.angular.z / 2.0
                elif self.light_color == "green":
                    if self.sign_name == "left":
                        print("Seeing left")
                        for linear_vel, left_angular_vel in left:
                            twist_msg = Twist()
                            twist_msg.linear.x = linear_vel
                            twist_msg.angular.z = left_angular_vel
                            self.pub.publish(twist_msg)
                            rate.sleep()
                    elif self.sign_name == "right":
                        print("Seeing right")
                        for linear_vel, right_angular_vel in right:
                            twist_msg = Twist()
                            twist_msg.linear.x = linear_vel
                            twist_msg.angular.z = right_angular_vel
                            self.pub.publish(twist_msg)
                            rate.sleep()
                    elif self.sign_name == "forward":
                        print("Seeing forward")
                        self.vel.linear.x = 0.3
                        self.vel.angular.z = 0.0
                        rospy.sleep(1)
            elif self.light_color == "Nothing":
                print("Nothing")
                self.vel.linear.x = self.behaviour_on_sign_vel.linear.x
                self.vel.angular.z = self.behaviour_on_sign_vel.angular.z
            else :
                print("NO conditions working")
            
            print("Esta chingadera no sirve")
            self.pub.publish(self.vel)
            rate.sleep()
            
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

    def behaviour_on_sign_vel_cb(self, vel) :
        self.behaviour_on_sign_vel = vel

    def sign_area_cb(self, sign_area) :
        self.sign_area = sign_area.data

    def sign_name_cb(self, sign_name) :
        self.sign_area = sign_name.data

    def light_color_cb(self, msg) :
        self.light_color = msg.data

    def shutdown(self) :
        print("LOGIC CONTROLLER NODE KILLED")
        self.pub.publish(self.zero)

if __name__ == "__main__" :
    rospy.init_node("Logic_Controller", anonymous = True)
    Logic_Controller()



