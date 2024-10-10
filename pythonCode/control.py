#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def control_servos():
    # Initialize the ROS node
    rospy.init_node('servo_controller', anonymous=True)
    
    # Create publishers for each servo
    pub_servo0 = rospy.Publisher('servo0_angle', Int16, queue_size=10)
    pub_servo1 = rospy.Publisher('servo1_angle', Int16, queue_size=10)
    pub_servo2 = rospy.Publisher('servo2_angle', Int16, queue_size=10)
    pub_servo3 = rospy.Publisher('servo3_angle', Int16, queue_size=10)
    pub_servo5 = rospy.Publisher('servo5_angle', Int16, queue_size=10)
    
    rate = rospy.Rate(0.5)  # Frequency of control messages
    
    while not rospy.is_shutdown():
        # Example sequence of angles
        pub_servo0.publish(10)
        pub_servo1.publish(65)
        pub_servo2.publish(120)
        pub_servo3.publish(100)
        pub_servo5.publish(30)
        
        rospy.sleep(2)
        
        pub_servo0.publish(150)
        pub_servo1.publish(30)
        pub_servo2.publish(90)
        pub_servo3.publish(150)
        pub_servo5.publish(80)
        
        rospy.sleep(2)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        control_servos()
    except rospy.ROSInterruptException:
        pass
