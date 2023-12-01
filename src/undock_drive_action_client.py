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
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# python standard library imports
from math import pi

# ros import
import roslib; roslib.load_manifest('undock_drive')
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

# own ros package imports
from undock_drive.msg import DockAction, DockFeedback, DockResult, DockGoal


def status_translator(status):
  if 0: print('')
  elif status == GoalStatus.PENDING   : state='PENDING'
  elif status == GoalStatus.ACTIVE    : state='ACTIVE'
  elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
  elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
  elif status == GoalStatus.ABORTED   : state='ABORTED'
  elif status == GoalStatus.REJECTED  : state='REJECTED'
  elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
  elif status == GoalStatus.RECALLING : state='RECALLING'
  elif status == GoalStatus.RECALLED  : state='RECALLED'
  elif status == GoalStatus.LOST      : state='LOST'
  return 'Status({} - {})'.format(status, state)


def undocking_cb(status, result):
    if result.status == GoalStatus.SUCCEEDED:
        rospy.loginfo("undocking_cb: Undocking succeded")
    elif result.status == GoalStatus.ABORTED:
        rospy.logwarn("undocking_cb: Undocking aborted")
    elif result.status == GoalStatus.LOST:
        rospy.logwarn("undocking_cb: Undocking lost")
    else:
        rospy.logwarn("undocking_cb: Unhandled Action response: {}".format(status_translator(result.status)))


def main():
    print("Init node")
    rospy.init_node('undock_drive_action_client')
    
    undock_drive_topic = rospy.get_param("~undock_drive_topic", "undock_drive_action") 
    undock_mode = rospy.get_param("~undock_mode", 0) 
    goal_x = rospy.get_param("~goal_x", 0.0) 
    goal_y = rospy.get_param("~goal_y", 0.0) 
    goal_yaw = rospy.get_param("~goal_yaw", -pi/2) 
    goal_trans_velocity = rospy.get_param("~goal_trans_velocity", 0.1) 
    goal_rot_velocity = rospy.get_param("~goal_rot_velocity", 1.0) 
    
    
    docking_client = actionlib.SimpleActionClient(undock_drive_topic, DockAction)
    while not docking_client.wait_for_server(rospy.Duration(5.0)):
        if rospy.is_shutdown(): return
        rospy.logwarn("Docking action server is not connected yet. still waiting...")
    
    # define goal
    goal = DockGoal()
    goal.undock_mode = undock_mode
    goal.x = goal_x
    goal.y = goal_y
    goal.yaw = goal_yaw
    goal.trans_velocity = goal_trans_velocity
    goal.rot_velocity = goal_rot_velocity
    
    docking_client.send_goal(goal, undocking_cb)
    rospy.on_shutdown(docking_client.cancel_goal)
    docking_client.wait_for_result()
    return docking_client.get_result()


if __name__ == "__main__":
    main()
