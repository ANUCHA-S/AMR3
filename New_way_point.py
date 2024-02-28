#!/usr/bin/env python
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist,Point
from ar_track_alvar_msgs.msg import AlvarMarkers

moveit_commander.roscpp_initialize(sys.argv)


waypoint = 0
rospy.init_node("simple_navigation_goals_and_ar_track_carrot_navigation_and_ur3_move")
move_base_client = SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to server')
move_base_client.wait_for_server()



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
group.set_max_velocity_scaling_factor(0.05)
group.set_max_acceleration_scaling_factor(0.03) 

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

group_variable_values = group.get_current_joint_values()

def getCarrot(msg):
    global carrot
    if len(msg.markers) == 1:
        carrot = msg.markers[0].pose.pose.position
    else:
        carrot = Point()



marker = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, getCarrot)
move = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

r = rospy.Rate(1)
ur3_move = 0
ar_enable = 0
back_enable = 0
N = 0
speed = Twist()

while not rospy.is_shutdown():
    for i in range (0,7,1):
        if waypoint == 0:   
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 4.420398991
            goal.target_pose.pose.position.y = -0.82794266222
            goal.target_pose.pose.orientation.w = 0.999996583114
            goal.target_pose.pose.orientation.z = 0.0105951038598
            rospy.loginfo('Sending goal_1')
            move_base_client.send_goal(goal)
            move_base_client.wait_for_result()


        if waypoint == 1:   
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 4.30685043281
            goal.target_pose.pose.position.y = -4.15354578526
            goal.target_pose.pose.orientation.w = 0.727780018108
            goal.target_pose.pose.orientation.z = -0.685810648243
            rospy.loginfo('Sending goal_2')
            move_base_client.send_goal(goal)
            move_base_client.wait_for_result()


        if waypoint == 2:   
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 0.454158551636
            goal.target_pose.pose.position.y = -4.30959163063
            goal.target_pose.pose.orientation.w = 0.713315057695
            goal.target_pose.pose.orientation.z = -0.700843512109
            rospy.loginfo('Sending goal_3')
            move_base_client.send_goal(goal)
            move_base_client.wait_for_result()
            


        if move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Hooray, the base moved meter forward')
            
            ar_enable = 1
        





    while not ar_enable == 0:
        if (carrot.z > 0.38) and (back_enable == 0)  and (ur3_move == 0) :
            speed.linear.x = carrot.z / 10
            speed.angular.z = -carrot.x / 2
            ur3_move = 0
            back_enable = 0
            move.publish(speed)
            rospy.loginfo(carrot.z)
            r.sleep()
        if (carrot.z < 0.38) and (carrot.z != 0) and (back_enable == 0) and (ur3_move == 0) :
            speed.linear.x = 0
            speed.angular.z = -0
            ar_enable = 0 
            # back_enable = 1 
            ur3_move = 1
        
            rospy.loginfo(carrot.z)
            # print (carrot.z)
            move.publish(speed)
            r.sleep()
            continue 



    while not ur3_move == 0:
        for i in range (0,1,1):
             if (N == 0) and (waypoint == 0) :
                 group_variable_values[2] = -1.1843388716327112     #1_elbow_joint
                 group_variable_values[1] = -1.9070966879474085      #2_- shoulder_lift_joint
                 group_variable_values[0] = -0.6196001211749476      #3_shoulder_pan_joint
                 group_variable_values[3] = -0.9576962629901331     #4_wrist_1_joint
                 group_variable_values[4] =  1.5699713230133057       #5_wrist_2_joint
                 group_variable_values[5] = 1.735373854637146       #6_wrist_3_joint
                 group.set_joint_value_target(group_variable_values)

                 plan2 = group.plan()
                 group.go(wait=True)

             if (N == 0) and (waypoint == 1) :
                 group_variable_values[2] = -1.6406243483172815     #1_elbow_joint
                 group_variable_values[1] = -1.7668669859515589      #2_- shoulder_lift_joint
                 group_variable_values[0] = -0.6158321539508265      #3_shoulder_pan_joint
                 group_variable_values[3] = -0.43885738054384404     #4_wrist_1_joint
                 group_variable_values[4] = 1.5986050367355347       #5_wrist_2_joint
                 group_variable_values[5] = 1.666211724281311       #6_wrist_3_joint
                 group.set_joint_value_target(group_variable_values)

                 plan2 = group.plan()
                 group.go(wait=True)

             if (N == 0) and (waypoint == 2) :
                 group_variable_values[2] = -1.1514518896686     #1_elbow_joint
                 group_variable_values[1] = -2.060317341481344      #2_- shoulder_lift_joint
                 group_variable_values[0] = -0.6240280310260218      #3_shoulder_pan_joint
                 group_variable_values[3] = -0.928927246724264     #4_wrist_1_joint
                 group_variable_values[4] = 1.6733360290527344       #5_wrist_2_joint
                 group_variable_values[5] = 1.610019564628601       #6_wrist_3_joint
                 group.set_joint_value_target(group_variable_values)

                 plan2 = group.plan()
                 group.go(wait=True)

    
             if N == 1:            
                 group_variable_values[2] = -0.758559528981344     #1_elbow_joint
                 group_variable_values[1] = -0.8946545759784144      #2_- shoulder_lift_joint
                 group_variable_values[0] = -0.892944637929098      #3_shoulder_pan_joint
                 group_variable_values[3] = -1.441965405141012      #4_wrist_1_joint
                 group_variable_values[4] = 1.6521508693695068       #5_wrist_2_joint
                 group_variable_values[5] =   1.6098642349243164      #6_wrist_3_joint
                 group.set_joint_value_target(group_variable_values)

                 plan2 = group.plan()
                 group.go(wait=True)

             if group.go() == True:
                N = N+1
                print(N)

             if N == 2:
               
               back_enable = 1
               ur3_move = 0
               N=0
               continue



    while not back_enable == 0:
        if (carrot.z < 1) and (carrot.z != 0) : # carrot regular is 1 meters
            speed.linear.x = -0.2
            move.publish(speed)
            rospy.loginfo(carrot.z)
            r.sleep()

        else:
            speed.linear.x = 0
            speed.angular.z = -0
            ar_enable = 0 
            back_enable = 0
            r.sleep()
            waypoint = waypoint + 1




        


      

        if waypoint == 3:
            waypoint = 0
            continue




    
        
   


