#!/usr/bin/env python

import rospy
from slamware_ros_sdk.srv import *

def get_mapping():
    rospy.init_node('sync_get_stcm', anonymous=True)
    rospy.loginfo('Start sync_get_stcm')

    rospy.wait_for_service('slamware_ros_sdk_server_node/sync_get_stcm', 5)

    # rospy.loginfo('wait_for_service ...!')

    try:
        sync_get_stcm = rospy.ServiceProxy('slamware_ros_sdk_server_node/sync_get_stcm', SyncGetStcm)
        rospy.loginfo('OK_for_service ...!')
        resp1 = sync_get_stcm()
        # rospy.loginfo('service data = %s',resp1)

        file = open('/home/rossi/catkin_ws/src/mine_robot_description/map/mapping_3floor.txt', 'w+')
        file.write(str(resp1))
        file.close()
        rospy.loginfo('Services call successfully : sync_get_stcm')

    except rospy.ServiceException as e:
        print("Service call failed!!: %s"%e)

if __name__ == '__main__':
    get_mapping()
