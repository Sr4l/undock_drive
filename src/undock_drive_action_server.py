#! /usr/bin/env python

# Copyright (c) 2015, Lars Kistner
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.


# python standard library imports
from math import sqrt, pi, atan2

# ros import
import roslib; roslib.load_manifest('undock_drive')
import rospy
import actionlib
import tf

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus

# own ros package imports
from undock_drive.msg import DockAction, DockFeedback, DockResult


class UndockDriveActionServer(object):
    def __init__(self, name):
        self.action_name = name
        self.global_frame = rospy.get_param("~global_frame", "/map")
        self.base_frame = rospy.get_param("~base_frame", "{}base_link".format(rospy.get_namespace()))
        velocity_cmd_topic = rospy.get_param("~velocity_cmd_topic", "mobile_base/commands/velocity")
        
        self.target_reached_distance = rospy.get_param("~target_reached_distance", 0.2) # m
        self.target_reached_angle = rospy.get_param("~target_reached_angle", 0.26) #rad (ca. 15degree)
        
        self.tflistener = tf.TransformListener()
        
        self.velocity_publisher = rospy.Publisher(velocity_cmd_topic, Twist, queue_size=2)
        
        self.action_server = actionlib.SimpleActionServer(self.action_name,
            DockAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()
    
    def get_position(self):
        """ Get the guessed robot position """
        try:
            now = rospy.Time.now()
            self.tflistener.waitForTransform(self.global_frame, self.base_frame, now, rospy.Duration(1.0))
            (trans,rot) = self.tflistener.lookupTransform(self.global_frame, self.base_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("{}: {}".format(self.action_name, e))
            return False
        
        x, y = trans[:2]
        orientation = euler_from_quaternion(rot)[2]
        
        return (x, y, orientation)
    
    def send_velocity_command(self, trans_x=0.0, rot_z=0.0):
        """ send a Twist message to mobile base """
        msg_vel = Twist()
        msg_vel.linear.x = trans_x
        msg_vel.angular.z =  rot_z
        self.velocity_publisher.publish(msg_vel)
    
    def travel_to_final_orientation(self, target_orientation, robot_orientation, rot_velocity):
        """
        calculate the delta angle and send out velocity commands to
        reduce the difference, as long as target_reached_angle is not reached
        return True if reached, False if not
        """
        delta_orientation = get_delta_orientation(target_orientation, robot_orientation)
        print delta_orientation, self.target_reached_angle
        if abs(delta_orientation) <= self.target_reached_angle:
            # we reached our target orientation stop
            return True
        
        rot_z = abs(rot_velocity) * delta_orientation
        self.send_velocity_command(0.0, rot_z)
        return False
        
    
    def distance_and_delta_orientation_to_goal(self, robot_pos, goal):
        """ calculate distance and delta angle to the goal"""
        # calculate distance to goal
        dx = robot_pos[0]- goal.x
        dy = robot_pos[1] - goal.y
        new_distance = sqrt(dx**2 + dy**2)
        
        # calculate orientation to goal
        yaw = atan2(dy, dx)
        dyaw = get_delta_orientation(yaw, robot_pos[2])
        
        return (new_distance, dyaw)
    
    def execute_cb(self, goal):
        rospy.loginfo("{}: New Call".format(self.action_name))
        
        rate = rospy.Rate(10)
        status = GoalStatus.PENDING
        
        # initialize distance variables
        new_distance = 0.0
        best_distance = float("inf")
        start_distance = False
        reached_goal = False
        
        # create messages
        feedback = DockFeedback()
        result = DockResult()
        
        while True:
            # check that preempt has not been requested by the client
            if self.action_server.is_preempt_requested():
                rospy.loginfo("{}: Preempted".format(self.action_name))
                self.send_velocity_command()
                self.action_server.set_preempted()
                break
            
            # try to get position of robot (tf lookup), abort action if not
            pos = self.get_position()
            if not pos:
                rospy.loginfo("{}: Can't get position".format(self.action_name))
                self.send_velocity_command()
                result.status = GoalStatus.ABORTED
                self.action_server.set_aborted(result)
                break
            
            new_distance, dyaw = self.distance_and_delta_orientation_to_goal(pos, goal)
            
            # if close to target set the reached_goal flag
            if not reached_goal and new_distance < self.target_reached_distance:
                reached_goal = True
            
            # set start distance to goal, if unset
            if not start_distance:
                start_distance = new_distance
            # if distance to goal is greater than the first distance to goal + 50% set robot to lost
            elif new_distance > start_distance * 1.5:
                rospy.loginfo("{}: Lost".format(self.action_name))
                self.send_velocity_command()
                result.status = GoalStatus.LOST
                self.action_server.set_aborted(result)
                break
            
            # we reached our goal as good as we can
            if best_distance < new_distance and reached_goal:
                if self.travel_to_final_orientation(goal.yaw, pos[2], goal.rot_velocity):
                    rospy.loginfo("{}: Succeeded".format(self.action_name))
                    self.send_velocity_command()
                    result.status = GoalStatus.SUCCEEDED
                    self.action_server.set_succeeded(result)
                    break
                else:
                    rospy.loginfo("{}: Need to travel to final orientation".format(self.action_name))
            # we are better than t-1, set new best_distance and travel for goal
            elif best_distance > new_distance:
                best_distance = new_distance
            
                # publish velocity command
                trans_x = -1 * abs(goal.trans_velocity)
                rot_z = abs(goal.rot_velocity) * dyaw * new_distance
                self.send_velocity_command(trans_x, rot_z)
                
                # publish the feedback
                feedback.status = GoalStatus.ACTIVE
                self.action_server.publish_feedback(feedback)
            
            rate.sleep()

def get_delta_orientation(goal_orientation, robot_orientation):
    """
    get the delta between two orientations, take pi = -pi into account
    """
    delta_orientation = goal_orientation - robot_orientation
    
    if delta_orientation > pi:
        delta_orientation -= 2*pi
    elif delta_orientation < -pi:
        delta_orientation += 2*pi
        
    return delta_orientation

def main():
    rospy.init_node("undock_drive_action")
    rospy.loginfo("Start {}".format(rospy.get_name()))
    UndockDriveActionServer(rospy.get_name())
    rospy.spin()
    rospy.loginfo("End {}".format(rospy.get_name()))

if __name__ == '__main__':
    main()
