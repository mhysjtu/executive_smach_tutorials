#!/usr/bin/env python
"""
Description:
    smach_viewer
    states are represented by ovals, and their outcomes are represented by directed edges. The outcomes of the container "sm_root" are shown in red.
    
    It is a good convention to capitalize state names, to write outcomes in lower case, and to use underscores where multiple words are present in a name
Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_02.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata: 
                []
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine terminating 'SPAWN':'succeeded':'succeeded'

"""

import rospy

import threading

import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer

import std_srvs.srv
import turtlesim.srv

def main():
    rospy.init_node('smach_usecase_step_02')

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm0:
        # Reset turtlesim
        StateMachine.add('RESET',
                ServiceState('reset', std_srvs.srv.Empty),
                {'succeeded':'SPAWN'})

        # Create a second turtle
        StateMachine.add('SPAWN',
                ServiceState('spawn', turtlesim.srv.Spawn,
                    request = turtlesim.srv.SpawnRequest(0.0,0.0,0.0,'turtle2')))

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/USE_CASE')
    sis.start()

    # Execute SMACH tree
    outcome = sm0.execute()

    # Signal ROS shutdown (kill threads in background)
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()
