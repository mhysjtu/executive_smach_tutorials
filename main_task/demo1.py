#!/usr/bin/env python
"""
Description:

Usage:
    $> roslaunch turtle_nodes.launch
    $> ./executive_step_04.py
"""

import rospy
import threading

import smach
from smach import StateMachine
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv


def main():
    rospy.init_node('hangdian_demo_1')

    # Construct static goals
    # polygon_big = turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0)
    # polygon_small = turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) 

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm0:

        # # check if sensors are OK
        # StateMachine.add('CHECK',
        #         ServiceState('hangdian/sensor_check', std_srvs.srv.Empty),
        #         {'succeeded':'SET_PARAM'})

        # # set param
        # StateMachine.add('SET_PARAM',
        #         ServiceState('hangdian/set_param', std_srvs.srv.Empty),
        #         {'succeeded':'YOLOICP'})

        # # yolo icp for button localization
        # StateMachine.add('YOLOICP',
        #         ServiceState('hangdian/teleport_absolute', std_srvs.srv.Empty),
        #         {'succeeded':'APPROACH'})

        # # arm approach panel
        # StateMachine.add('APPROACH',
        #         ServiceState('hangdian/iiwa/move_to_cartesian_pose_lin', std_srvs.srv.Empty),
        #         {'succeeded':'IMPEDENCE_MODE'})

        # # arm change to impedence mode 
        # StateMachine.add('IMPEDENCE_MODE',
        #         ServiceState('hangdian/iiwa/configuration/ConfigureControlMode', std_srvs.srv.Empty),
        #         {'succeeded':'GRIPPER_CLOSE'})



        # check if sensors are OK
        StateMachine.add('CHECK',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'SET_PARAM'})

        # set param
        StateMachine.add('SET_PARAM',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'OBSERVE_POSITION'})

        # arm approach panel
        StateMachine.add('OBSERVE_POSITION',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'YOLOICP'})

        # yolo icp for button localization
        StateMachine.add('YOLOICP',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'APPROACH'})

        # arm approach panel
        StateMachine.add('APPROACH',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'IMPEDENCE_MODE'})

        # arm change to impedence mode 
        StateMachine.add('IMPEDENCE_MODE',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'GRIPPER_CLOSE'})




        # close gripper
        StateMachine.add('GRIPPER_CLOSE',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'GRIPPER_ROTATE'})

        # rotate gripper
        StateMachine.add('GRIPPER_ROTATE',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'GRIPPER_OPEN'})

        # open gripper
        StateMachine.add('GRIPPER_OPEN',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'POSITION_MODE'})
        



        #arm change to position mode 
        StateMachine.add('POSITION_MODE',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'LEAVE'})
        
        # arm go to observe position
        StateMachine.add('LEAVE',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'TASK_FINISHED'})
        
        # arm go to observe position
        StateMachine.add('TASK_FINISHED',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'aborted':'OBSERVE_POSITION', 'succeeded':'succeeded', 'preempted':'preempted'})




        # # arm change to position mode 
        # StateMachine.add('POSITION_MODE',
        #         ServiceState('hangdian/iiwa/configuration/ConfigureControlMode', std_srvs.srv.Empty),
        #         {'succeeded':'LEAVE'})
        
        # # arm go to observe position
        # StateMachine.add('LEAVE',
        #         ServiceState('hangdian/iiwa/move_to_cartesian_pose_lin', std_srvs.srv.Empty),
        #         {'succeeded':'TASK_FINISHED'})
        
        # # arm go to observe position
        # StateMachine.add('TASK_FINISHED',
        #         ServiceState('hangdian/task_check', std_srvs.srv.Empty),
        #         {'aborted':'APPROACH'})


    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_demo_1', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
