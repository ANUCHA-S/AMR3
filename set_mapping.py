#!/usr/bin/env python

import rospy
from slamware_ros_sdk.srv import *
from geometry_msgs.msg import Pose
import numpy as np

def set_mapping():
    rospy.init_node('sync_set_stcm', anonymous=True)
    rospy.loginfo('Start sync_set_stcm')

    rospy.wait_for_service('slamware_ros_sdk_server_node/sync_set_stcm', 5)
    # rospy.loginfo('wait_for_service ...!')

    try:
        file = open('/home/rossi/catkin_ws/src/mine_robot_description/map/mapping_3floor.txt', 'r') #mapping_floor3, mapping_44304
        resp1 = file.read()

        resp2 = resp1.split('[', 1)
        resp3 = resp2[1].split(']')
        resp4 = resp3[0].split(', ')
        for i in range(0, len(resp4)): 
            resp4[i] = int(resp4[i])

        # rospy.loginfo('service data = %s',resp4)
        # rospy.loginfo('OK_read_txt ...!')

        robot_pose = Pose()
        robot_pose.position.x = 0.0
        robot_pose.position.y = 0.0
        robot_pose.position.z = 0.0
        robot_pose.orientation.w = 1.0
        robot_pose.orientation.x = 0.0
        robot_pose.orientation.x = 0.0
        robot_pose.orientation.z = 0.0

        sync_set_stcm = rospy.ServiceProxy('slamware_ros_sdk_server_node/sync_set_stcm', SyncSetStcm)
        # rospy.loginfo('OK_for_service ...!')


        resp2 = sync_set_stcm(resp4, robot_pose)
        rospy.loginfo('Services call successfully : sync_set_stcm')

        file.close()

    except rospy.ServiceException as e:
        print("Service call failed >>> : %s"%e)

if __name__ == '__main__':
    set_mapping()
