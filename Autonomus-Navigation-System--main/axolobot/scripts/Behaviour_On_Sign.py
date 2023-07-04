#!/usr/bin/env python 
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class Behaviour_On_Sign() :
    def __init__(self) :
        rospy.on_shutdown(self.shutdown)

        node_rate = 10
        rate = rospy.Rate(node_rate)

        rospy.Subscriber("/traffic/sign_name", String, self.sign_name_cb)
        rospy.Subscriber("/vel/line_follower", Twist, self.line_follower_vel_cb)
        rospy.Subscriber("/traffic/sign_area", Float32, self.sign_area_cb)

        self.sign_flag_pub = rospy.Publisher("/sign_on", Int8, queue_size = 1)
        self.sign_vel_pub = rospy.Publisher("/vel/behaviour_on_sign", Twist, queue_size = 1)
        self.warning_pub = rospy.Publisher("/warning/behaviour_on_sign", String, queue_size = 1)

        self.line_follower_vel = Twist()
        self.sign_vel =  Twist()
        self.zero =  Twist()

        self.sign_name = ""
        self.sign_area = 0.0
        self.sign_on = 0

        i = 0
        k = 0
        warning_sign = 0

        while not rospy.is_shutdown() :
            if self.sign_on :
                if i == k :
                    warning_sign = 0
                    self.sign_on = 0
                    self.sign_vel.linear.x = self.zero.linear.x
                    self.sign_vel.angular.z = self.zero.angular.z
                else :
                    if warning_sign :
                        self.sign_vel.linear.x = self.line_follower_vel.linear.x / 2.0
                        self.sign_vel.angular.z = self.line_follower_vel.angular.z / 2.0
                    i += 1
                    self.sign_vel_pub.publish(self.sign_vel)
            else :
                if self.sign_area > 2040.0:
                    if self.sign_name == "stop" :
                        k = 100
                        self.sign_on = 1
                        self.sign_vel.linear.x = self.zero.linear.x
                        self.sign_vel.angular.z = self.zero.angular.z
                    elif self.sign_name == "construction" :
                        k = 20
                        self.sign_on = 1
                        warning_sign = 1
                    elif self.sign_name == "give_way" :
                        k = 50
                        self.sign_on = 1
                        warning_sign = 1
                elif self.sign_name == "Nothing" :
                        self.sign_vel.linear.x = self.line_follower_vel.linear.x
                        self.sign_vel.angular.z = self.line_follower_vel.angular.z
                else :
                    print("NO conditions working")
            rate.sleep()

    def line_follower_vel_cb(self, vel) :
        self.line_follower_vel = vel

    def sign_name_cb(self, msg) :
        self.sign_name = msg.data

    def sign_area_cb(self, sign_area) :
        self.sign_area = sign_area.data

    def shutdown(self) :
        self.sign_vel_pub.publish(self.zero)
        print("BEHAVIOUR ON SIGN NODE KILLED")

if __name__ == "__main__" :
    rospy.init_node("Behaviour_On_Sign", anonymous = True)
    Behaviour_On_Sign()





