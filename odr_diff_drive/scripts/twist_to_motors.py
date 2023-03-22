#!/usr/bin/env python
from re import S
from turtle import left
#from typing_extensions import dataclass_transform
import rospy
import roslib
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from odr_diff_drive.msg import wheel_velocity

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    
    def joy_to_cmd_vel(self,msg):
        
    
        if(msg.axes[1]==0 and msg.axes[0]==0 and msg.axes[1]==0 ):
            self.dx=0
            self.dr = 0
            self.right = 1.0 * self.dx + self.dr * self.w / 2
            #right=self.right
            #print("right",right)
            self.left = 1.0 * self.dx - self.dr * self.w / 2
            #left=self.left
          
        if (msg.buttons[5]==1):
            if (msg.axes[1]>0):
                self.dx=min(480,480*msg.axes[1])
                self.dr = 0
                self.right = 1.0 * self.dx + self.dr * self.w / 2
                #right=self.right
                #print("right",right)
                self.left = 1.0 * self.dx - self.dr * self.w / 2
                #left=self.left


            elif(msg.axes[1]<0):
                self.dx=max(-480,480*msg.axes[1])
                self.dr = 0
                self.right = 1.0 * self.dx + self.dr * self.w / 2
                #right=self.right
                #print("right",right)
                self.left = 1.0 * self.dx - self.dr * self.w / 2
                #left=self.left





            if (msg.axes[0]<0):
                self.dx=min(480,-480*msg.axes[0])
                self.dr = 0
                #self.right = 1.0 * self.dx + self.dr * self.w / 2
                self.left =0
                #right=self.right
                #print("right",right)
                self.right = 1.0 * self.dx - self.dr * self.w / 2
                #left=self.left


            elif(msg.axes[0]>0):
                self.dx=min(480,480*msg.axes[0])
                self.dr = 0

                self.left = 1.0 * self.dx + self.dr * self.w / 2
                #right=self.right
                #print("right",right)
                #self.left = 1.0 * self.dx - self.dr * self.w / 2
                self.right = 0

            if(msg.axes[6]>0):
                self.dx=20
                self.dr=0
                self.right = -1.0 * self.dx + self.dr * self.w / 2
                #right=self.right
                #print("right",right)
                self.left = 1.0 * self.dx - self.dr * self.w / 2

                
            elif(msg.axes[6]<0):
                self.dx=-20
                self.dr=0
                self.right = -1.0 * self.dx + self.dr * self.w / 2
                #right=self.right
                #print("right",right)
                self.left = 1.0 * self.dx - self.dr * self.w / 2
            
        #elif(msg.buttons[5]==0):
         #   self.left=0
          #  self.right=0

            #print("left",left)
            # rospy.loginfo("publishing: (%d, %d)", left, right) 
            self.right=self.translate(self.right,-480,480,-2400,2400)
            if(self.right>4800 ):
                self.right=4800 
            elif (self.right<-4800):
                self.right=-4800
            print("===========")
            #print(self.right)
            self.left=self.translate(self.left,-480,480,-2400,2400)
            if(self.left>4800 ):
                self.left=2400  
            elif (self.left<-4800):
                self.left=-2400
            self.wheel_vel.Vl.data=self.left
            self.wheel_vel.Vr.data=self.right    
            #self.pub_lmotor.publish(self.left)
            #self.pub_rmotor.publish(self.right)
            self.pub_motor.publish(self.wheel_vel)   
            self.ticks_since_target += 1

    
    
    
    def __init__(self):
    #############################################################
        rospy.init_node("joy_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        
        self.wheel_vel=wheel_velocity()
        
        self.w = rospy.get_param("~base_width", 0.45)
    
        #self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Int16, queue_size=10)
        #self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Int16, queue_size=10)
        self.pub_motor = rospy.Publisher('wheel_target_vel', wheel_velocity, queue_size=10)
        #rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        rospy.Subscriber("joy", Joy, self.joy_to_cmd_vel)
    
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0

    def translate(self,value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        #print(leftSpan)
        rightSpan = rightMax - rightMin
        #print(rightSpan)

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)
        #print(valueScaled)

        # Convert the 0-1 range into a value in the right range.
        #print(rightMin + (valueScaled * rightSpan))
        return (rightMin + (valueScaled * rightSpan))
    

    




    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(100)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks+1:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        #self.right = 1.0 * self.dx + self.dr * self.w / 2
        #right=self.right
        #print("right",right)
        #self.left = 1.0 * self.dx - self.dr * self.w / 2
        #left=self.left
        #print("left",left)
        # rospy.loginfo("publishing: (%d, %d)", left, right) 
        #self.right=self.translate(right,-480,480,-4800,4800)
        # if(self.right>4800 ):
        #     self.right=4800 
        # elif (self.right<-4800):
        #     self.right=-4800
        # print("===========")
        # print(self.right)
        # self.left=self.translate(left,-480,480,-4800,4800)
        # if(self.left>4800 ):
        #     self.left=4800  
        # elif (self.left<-4800):
        #     self.left=-4800
        # self.wheel_vel.Vl.data=self.left
        # self.wheel_vel.Vr.data=self.right    
        #self.pub_lmotor.publish(self.left)
        #self.pub_rmotor.publish(self.right)
        print("---------------------------")
        self.pub_motor.publish(self.wheel_vel)   
        self.ticks_since_target += 1

    #############################################################
    #def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        #self.ticks_since_target = 0
        #self.dx = msg.linear.x
        #self.dr = msg.angular.z
        #self.dy = msg.linear.y

    
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
