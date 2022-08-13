#!/usr/bin/env python
import time
import csv

from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

drone_position_x = 0
drone_position_y = 0
drone_position_z = 0
target_position_x = 0
target_position_y = 0
waktu = 0

header = ['/clock', '/droneX', '/droneY', '/droneZ', '/TargetX', '/TargetY']
f = open('/home/patricknks/data.csv', 'w')
writer = csv.writer(f)
writer.writerow(header)

def dronepos_cb(msg):
    global drone_position_x, drone_position_y
    drone_position_x = msg.pose.position.x
    drone_position_y = msg.pose.position.y

def tfmini_cb(msg):
    global drone_position_z
    drone_position_z = msg.range

def ltpos_cb(msg):
    global target_position_x, target_position_y
    target_position_x = msg.pose.position.x
    target_position_y = msg.pose.position.y

def clock_cb(msg):
    global waktu
    waktu = msg.clock

def main():
    try:
        counter = 0
        rospy.init_node("write_data")
        rate = rospy.Rate(15)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, dronepos_cb)
        rospy.Subscriber('/tfmini', Range, tfmini_cb)
        rospy.Subscriber('/est_dist', PoseStamped, ltpos_cb)
        rospy.Subscriber('/clock', Clock, clock_cb)

        pub=rospy.Publisher('/log',Float64,queue_size=10)

        while not rospy.is_shutdown():
            counter = counter + 1
            pub.publish(counter)
            rospy.loginfo('{}, {}, {}, {}, {}, {}'.format(waktu, drone_position_x, drone_position_y, drone_position_z, target_position_x, target_position_y))
            writer.writerow([waktu, drone_position_x, drone_position_y, drone_position_z, target_position_x, target_position_y])
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

    f.close()

if __name__ == '__main__':
    main()