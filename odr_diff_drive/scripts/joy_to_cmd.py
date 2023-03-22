   #! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot
# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    if (abs(data.axes[1])>0 ||  abs(data.axes[0])>0):
        
        twist.linear.x = max(0, 48*data.axes[1])
            
        twist.angular.z = max(0,48*data.axes[0])
        pub.publish(twist)

# Intializes everything
def start():
    
    global pub
    pub = rospy.Publisher('cmd_vel', Twist)
        # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
# starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()
if __name__ == '__main__':
start()