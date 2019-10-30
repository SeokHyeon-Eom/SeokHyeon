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
    speed = 110
    distance = 35
    diff = 3.5

    while cnt < 6:
        if speed > 130:
            speed = 130
        if direc == 'front':
            if usonic_data[1] <= distance:
                drive(90, 90)
                time.sleep(5)
                for i in range(2):
                    drive(90,90)
                    time.sleep(0.1)
                    drive(90+diff,60)
                    time.sleep(0.1)
                cnt += 1
                direc = 'back'
            else:
	        drive(90, speed)
        elif direc == 'back':
            if usonic_data[4] <= 42:
                drive(90, 90)
                time.sleep(5)
                cnt += 1
                if cnt == 4:
                    distance += 27
                elif cnt == 2:
                    distance += 20
                speed += 10
                direc = 'front'
            else:
                drive(90 + diff, 70)
        rate.sleep()
    rospy.on_shutdown(exit_node)
