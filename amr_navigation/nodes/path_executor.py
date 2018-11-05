#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, MoveToResult, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult


class PathExecutor: 
    def __init__(self): 
       #initalizing the action to execute path and starting the server
        self._ep = SimpleActionServer('/path_executor/execute_path', 
            ExecutePathAction, execute_cb=self.execute_cb, 
            auto_start = False)
              
        self._ep.start()
               
        
        self.publisher = rospy.Publisher('/path_executor/current_path',Path, queue_size = 10)
        self.success = True
        self.aborted = False
         # creating messages to publish feedback and result
        self.feedback = ExecutePathFeedback()
        self.result   = ExecutePathResult()
        
    def execute_cb(self, goal):
        rospy.loginfo("ExecutePathAction received")         
       
       
        r = rospy.Rate(10)
      
        self.action_goal = MoveToAction
        
        #Get obstacle avoidance
        self.obstacle_avoidance = rospy.get_param('/path_executor/use_obstacle_avoidance')
  
        #Choose server for move_to depending on obstacle avoidance
        if self.obstacle_avoidance : 
            rospy.loginfo("obstacle_avoidance = True")
            move_client = SimpleActionClient('/bug2/move_to', MoveToAction)
        else:
            rospy.loginfo("obstacle_avoidance = False")                
            move_client = SimpleActionClient('/motion_controller/move_to', MoveToAction)

        move_client.wait_for_server()            
            
        #Publish path to /path_executor/current_path
        self.publisher.publish(goal.path)
        
        #Iterate through path
        for index in range(len(goal.path.poses)) :  
            rospy.loginfo("\nGoal received")         
            rospy.loginfo("Goal: \n{0}".format(goal.path.poses[index].pose))
            
            #Publish goal pose to move_to server
            self.action_goal.target_pose = goal.path.poses[index]
            move_client.send_goal(self.action_goal)  
            
                
            while not rospy.is_shutdown() :
                if self._ep.is_preempt_requested():
                    rospy.logwarn('There is preemed state requested')
                    move_client.cancel_all_goals()
                    self._ep.set_preempted()                    
                    self.success = False
                    self.aborted = True
                    break
                
                #get result state from move_to server
                state = move_client.get_state()

                #Success
                if state == 3:
                    rospy.loginfo("Goal reached")
                    #Publish the feedback
                    self.feedback.pose = self.action_goal.target_pose
                    self.feedback.reached = True
                    self._ep.publish_feedback(self.feedback)
                    #Store to visited poses
                    self.result.visited.append(True)
                    break
                #Unreachable goal
                elif state == 4:
                    rospy.logwarn("Goal is unreachable")
                    #Skip unreachable
                    if goal.skip_unreachable:

                        #Publish the feedback
                        self.feedback.pose =self.action_goal.target_pose
                        self.feedback.reached = False
                        self._ep.publish_feedback(self.feedback)
                        #Storing the visted 
                        self.result.visited.append(False)
                        break
                    #Abort                    
                    else:
                        move_client.cancel_all_goals()
                        self._ep.set_aborted()      
                        self.success = False
                        self.aborted = True
                        break
                    
                r.sleep()
                
            if self.aborted:
                break
                
        
        if self.success:        
            self._ep.set_succeeded(self.result)
            rospy.loginfo("Path finished with success\n")
                    

    def move_to_done_cb(self, state, result):
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()