#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray

motor_pub = None
usonic_data = None

def init_node():
    global motor_pub
    rospy.init_node('sample')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg',
                                Int32MultiArray, queue_size=1)

def exit_node():
    print('finished')

def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)

def callback(data):
    global usonic_data
    usonic_data = data.data

if __name__ == '__main__':
    init_node()
    time.sleep(3)
    
    rate = rospy.Rate(10)

    direc = 'front'
    cnt = 0   
    while not rospy.is_shutdown():
        if direc == 'front':
            if usonic_data[1] <= 60:
                direc = 'back'
                cnt = 1
            else:
                drive(90, 110)
	
        elif direc == 'back':
            if usonic_data[4] <= 60:
                direc = 'front'
            else:    
                if cnt == 1:
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, 70)
                    time.sleep(0.1)
                    cnt = 0
                else:
                    drive(90, 70)
        rate.sleep()
    rospy.on_shutdown(exit_node)


