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
    while not rospy.is_shutdown():
        speed = 110
        if direc == 'front':
            drive(90, speed)
            if usonic_data[1] <= 30:
                for i in range(50):
                    drive(90, 90)
                    time.sleep(0.1)
                for i in range(2):
                    drive(90,90)
                    time.sleep(0.1)
                    drive(90,70)
                    time.sleep(0.1)
                direc = 'back'
	
        elif direc == 'back':
            drive(90, 70)
            if usonic_data[4] <= 30:
                for i in range(50):
                    drive(90, 90)
                    time.sleep(0.1)
                direc = 'front'
                if speed < 130:
                    speed += 10

        rate.sleep()
    rospy.on_shutdown(exit_node)
