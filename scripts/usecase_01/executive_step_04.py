#!/usr/bin/env python
"""
Description:
    mhy add more comment to understand some function better
Usage:
    $> roslaunch turtle_nodes.launch
    # must map "cmd_vel" to right namespace
    $> ./executive_step_04.py

Output:
    [INFO] : State machine starting in initial state 'RESET' with userdata: 
                []
    [INFO] : State machine transitioning 'RESET':'succeeded'-->'SPAWN'
    [INFO] : State machine transitioning 'SPAWN':'succeeded'-->'TELEPORT1'
    [INFO] : State machine transitioning 'TELEPORT1':'succeeded'-->'TELEPORT2'
    [INFO] : State machine transitioning 'TELEPORT2':'succeeded'-->'BIG'
    [WARN] : Still waiting for action server 'turtle_shape1' to start... is it running?
    [INFO] : Connected to action server 'turtle_shape1'.
    [INFO] 1279655938.783058: State machine transitioning 'BIG':'succeeded'-->'SMALL'
    [INFO] 1279655975.025202: State machine terminating 'SMALL':'succeeded':'succeeded'

"""

import rospy

import threading

import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
import turtlesim.srv
import turtle_actionlib.msg


def main():
    rospy.init_node('smach_usecase_step_04')

    # Construct static goals
    polygon_big = turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0)
    polygon_small = turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 1.0) 

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
                    request = turtlesim.srv.SpawnRequest(5.0,0.0,0.0,'turtle2')),
                {'succeeded':'TELEPORT1'})

        # Teleport turtle 1
        StateMachine.add('TELEPORT1',
                ServiceState('turtle1/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(5.0,1.0,0.0)),
                {'succeeded':'TELEPORT2'})

        # Teleport turtle 2
        StateMachine.add('TELEPORT2',
                ServiceState('turtle2/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(5.0,5.0,0.0)),
                {'succeeded':'BIG'})

        # Draw a large polygon with the first turtle
        StateMachine.add('BIG',
                SimpleActionState('/turtle_shape1',turtle_actionlib.msg.ShapeAction,
                    goal = polygon_small),
                {'succeeded':'SMALL'})

        # Draw a small polygon with the second turtle
        StateMachine.add('SMALL',
                SimpleActionState('/turtle_shape2',turtle_actionlib.msg.ShapeAction,
                    goal = polygon_small))
        # if no {:} is defined, succ to succ, abor to abor, pree to pree; if only {succ:succ}, no other two mappings

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/USE_CASE')
    sis.start()

    # # Execute SMACH tree
    # outcome = sm0.execute()

    # # Signal ROS shutdown (kill threads in background)
    # rospy.spin()
    
    # Set preempt handler
    # Sets a ROS pre-shutdown handler to preempt a given SMACH container when ROS receives a shutdown request.
    # This can be attached to multiple containers, but only needs to be used on the top-level containers.
    # state machine to "preempted" result
    # if not added, the sm cannot stop when ctrl-c is pressed
    # you can put it after execute in another thread
    set_preempt_handler(sm0)


    # # only this cannot ctrl-c the script while the sm is not ended
    # # should put sm0 in another thread
    # outcome = sm0.execute()

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()
    

    # Signal handler
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()
