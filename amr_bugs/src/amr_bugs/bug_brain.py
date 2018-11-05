#!/usr/bin/env python
from planar import Point, Vec2
from planar.c import Line
from math import degrees

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'

class BugBrain:

    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        self.goal_x= goal_x
        self.goal_y=goal_y
        self.wp_goal_point=Point(self.goal_x,self.goal_y)
        self.side=side
        self.pass_points=[]
        self.c=0
        self.f=False
        self.start_point=None


    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        self.x=x
        self.y=y
        self.theta=theta
        self.wp_start_point=Point(self.x,self.y)
        #making the list passed of points
        self.pass_points.append((x,y))
        self.vec_at_point=Point(x,y)
        self.angle_vector= self.wp_goal_point-self.vec_at_point
        #making line from the point to the goal 
        self.ln_goal_line=Line.from_points([self.wp_start_point,self.wp_goal_point])
        #calculating distance from the point to goal
        self.distance_to_goal=self.vec_at_point.distance_to(self.wp_goal_point)
        self.c=self.c+1
      

    def leave_wall(self, x, y, theta):
        """ 
        This function is called when the state machine leaves the wallfollower
        state.
        """
        self.f=True
        self.wp_leave_points=Point(x,y)

        # compute and store necessary variables
       

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
       # wp_p1 = Point(x,y)
        #if (self.wp_start_point==wp_p1):
            #return True
        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        self.x_curr=x
        self.y_curr=y

        v_current=Vec2(x,y)
        goal_vector=self.wp_goal_point
        v_1= v_current-goal_vector
        #distance between current pose and goal vector
        self.current_to_goal=v_current.distance_to(goal_vector)
        #finding the angle between the current point and goal
        angle_to_goal=v_1.angle_to(self.angle_vector)

        #if the point is nearing to the goal
        if(abs(self.x-self.x_curr)>0.3 or abs(self.y-self.y_curr)>0.3)and (abs(angle_to_goal)< 2 or(abs(angle_to_goal)< 182 and abs(angle_to_goal)>178) ):
            #checking if the point lie on the line connecting the start point and goal line if so leaving the wall
            for i in self.pass_points:
                if self.ln_goal_line.contains_point(i):
                    return True
            
        return False

#==============================================================================
