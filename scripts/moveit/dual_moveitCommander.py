#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject, AttachedCollisionObject, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler

import numpy as np
import math




class MoveIt_Control:
    def __init__(self, is_use_gripper=False):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.right_arm = moveit_commander.MoveGroupCommander('right_gluon_manipulator')
        self.left_arm = moveit_commander.MoveGroupCommander('left_gluon_manipulator')
        self.right_arm.set_goal_joint_tolerance(0.005)
        self.right_arm.set_goal_position_tolerance(0.005)
        self.right_arm.set_goal_orientation_tolerance(0.005)
        self.left_arm.set_goal_joint_tolerance(0.005)
        self.left_arm.set_goal_position_tolerance(0.005)
        self.left_arm.set_goal_orientation_tolerance(0.005)

        self.reference_frame = 'base_link'

        self.right_end_effector_link = 'right_6_Link'
        self.right_reference_frame = 'base_link'
        self.right_arm.set_pose_reference_frame(self.right_reference_frame)
        self.left_end_effector_link = 'left_6_Link'
        self.left_reference_frame = 'base_link'
        self.left_arm.set_pose_reference_frame(self.left_reference_frame)
        print("============ Right Reference frame: %s" % self.right_arm.get_planning_frame())
        print("============ Left Reference frame: %s" % self.left_arm.get_planning_frame())
        print("============ Robot Groups:")
        print(self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state().joint_state.position[7])
        print("============")
        self.right_arm.set_planning_time(5)
        self.right_arm.allow_replanning(True)
        self.right_arm.set_planner_id('RRTConnect')
        self.left_arm.set_planning_time(5)
        self.left_arm.allow_replanning(True)
        self.left_arm.set_planner_id('RRTConnect')

        self.right_arm.set_max_acceleration_scaling_factor(1)
        self.right_arm.set_max_velocity_scaling_factor(1)
        self.left_arm.set_max_acceleration_scaling_factor(1)
        self.left_arm.set_max_velocity_scaling_factor(1)

 #       self.right_go_home()
 #       self.left_go_home()

        self.set_scene()

#        self.testRobot()


    def close(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    def testRobot(self):
        try:
            print("Test for robot...\n")
            self.right_go_handup()
            self.left_go_handup()
            rospy.sleep(2)
            self.right_go_home()
            self.left_go_home()
#            rospy.sleep(2)
#            self.move_j([0.3,-1.5,1.2,0.0,-1,0.454125],a=0.5,v=0.5)
#            rospy.sleep(2)
#            self.go_home()
#            rospy.sleep(2)
#            self.move_p([0.121,-0.296,0.372,-0.001,-1.189,-1.571])
#            rospy.sleep(2)
        except:
            print("Test fail! \n")

    

    def set_scene(self):
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)
        table_id = 'table'
        self.scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_size = [2,2,0.01]
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -table_size[2]/2-0.15
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(table_id, table_pose, table_size)
     #   self.setClolor(table_id, 0.5, 0.5, 0.5, 1.0)
      #  self.sendColors()


    
    def right_move_joint(self, joint_configuration=None, a=1, v=1):
        if joint_configuration==None:
            joint_configuration = [0,-1.5707,0,-1.5707,0,0]
        self.right_arm.set_max_acceleration_scaling_factor(a)
        self.right_arm.set_max_velocity_scaling_factor(v)
     #   self.right_arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.right_arm.go(joint_configuration,wait=True)
        self.right_arm.stop()
        rospy.sleep(1)

    

    def left_move_joint(self, joint_configuration=None, a=1, v=1):
        if joint_configuration==None:
            joint_configuration = [0,-1.5707,0,-1.5707,0,0]
        self.left_arm.set_max_acceleration_scaling_factor(a)
        self.left_arm.set_max_velocity_scaling_factor(v)
     #   self.right_arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.left_arm.go(joint_configuration,wait=True)
        self.left_arm.stop()
        rospy.sleep(1)


    def right_move_position(self, tool_configuration=None, a=1, v=1, is_wait=True):
        if tool_configuration==None:
            tool_configuration = [0,0,0.3,0,-np.pi/2,0]
        self.right_arm.set_max_acceleration_scaling_factor(a)
        self.right_arm.set_max_velocity_scaling_factor(v)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.right_reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = tool_configuration[0]
        target_pose.pose.position.y = tool_configuration[1]
        target_pose.pose.position.z = tool_configuration[2]
        q = quaternion_from_euler(tool_configuration[3], tool_configuration[4], tool_configuration[5])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.right_arm.set_start_state_to_current_state()
        self.right_arm.set_pose_target(target_pose, self.right_end_effector_link)
        rospy.loginfo("right_move_p:"+ str(tool_configuration))
        # plan = self.right_arm.plan()
        # print(plan[0])
        # while(plan[0]==False):
        #     print("planning failed, retry with a ittle closer target!")
        #     target_pose.pose.position.y += 0.02
        #     plan = self.right_arm.plan()
        #     print(plan[0])
        # print("planed")
        self.right_arm.go(wait=is_wait)
        
        # self.right_arm.stop()
        print("excuted")
        # self.right_arm.clear_pose_targets()
       # rospy.sleep(1)





    def left_move_position(self, tool_configuration=None, a=1, v=1, is_wait=True):
        if tool_configuration==None:
            tool_configuration = [0,0,0.3,0,-np.pi/2,0]
        self.left_arm.set_max_acceleration_scaling_factor(a)
        self.left_arm.set_max_velocity_scaling_factor(v)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.left_reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = tool_configuration[0]
        target_pose.pose.position.y = tool_configuration[1]
        target_pose.pose.position.z = tool_configuration[2]
        q = quaternion_from_euler(tool_configuration[3], tool_configuration[4], tool_configuration[5])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.left_arm.set_start_state_to_current_state()
        self.left_arm.set_pose_target(target_pose, self.left_end_effector_link)
        rospy.loginfo("left_move_p:"+ str(tool_configuration))
        # plan = self.left_arm.plan()
        # print(plan[0])
        # while(plan[0]==False):
        #     print("planning failed, retry with a little closer target!")
        #     target_pose.pose.position.y += 0.02
        #     plan = self.left_arm.plan()
        #     print(plan[0])
        # print("planed")
        self.left_arm.go(wait=is_wait)
        
        # self.left_arm.stop()
        print("excuted")
        # self.left_arm.clear_pose_targets()
        #rospy.sleep(1)





    def right_go_home(self,a = 1,v = 1,is_wait = True):
        self.right_arm.set_max_acceleration_scaling_factor(a)
        self.right_arm.set_max_velocity_scaling_factor(v)
        self.right_arm.set_named_target("right_home")
        self.right_arm.go(wait=is_wait)
        


    def left_go_home(self,a = 1,v = 1, is_wait = True):
        self.left_arm.set_max_acceleration_scaling_factor(a)
        self.left_arm.set_max_velocity_scaling_factor(v)
        self.left_arm.set_named_target("left_home")
        self.left_arm.go(wait=is_wait)
        

    def right_go_handup(self,a = 1,v = 1, is_wait = True):
        self.right_arm.set_max_acceleration_scaling_factor(a)
        self.right_arm.set_max_velocity_scaling_factor(v)
        self.right_arm.set_named_target("right_handup")
        self.right_arm.go(wait=is_wait)
        

    def left_go_handup(self,a = 1,v = 1, is_wait=True):
        self.left_arm.set_max_acceleration_scaling_factor(a)
        self.left_arm.set_max_velocity_scaling_factor(v)
        self.left_arm.set_named_target("left_handup")
        self.left_arm.go(wait=is_wait)
        


if __name__ =="__main__":
    rospy.init_node("dual_moveit_commander", anonymous=True)
    moveit_server = MoveIt_Control(is_use_gripper=False)
    # moveit_server.left_move_position([-0.12328994383335776, -0.3853917228570018, -0.10, 1.5707963267948966, 0, -1.5707963267948966])
    # moveit_server.right_move_position([0.12328994383335776, -0.3853917228570018, -0.10, -1.5707963267948966, 0, -1.5707963267948966])
    #rospy.spin()