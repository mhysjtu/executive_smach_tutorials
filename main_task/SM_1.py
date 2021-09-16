#!/usr/bin/env python
#-*- coding: UTF-8 -*-
"""
Description:

Usage:
    $> cd ~/aircraft_ws/src/hangdian_detection/hangdian_arm_controllers/gripper_controllers/scripts
    $> python empty_service_node.py
    in this directory:
        $> python SM_1.py
        $> rosrun smach_viewer smach_viewer.py
"""

import rospy
import threading

import smach
from smach import StateMachine, State
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
from gripper_msgs.srv import gripper_control, gripper_controlRequest
from iiwa_msgs.msg import ControlMode
from iiwa_msgs.srv import ConfigureControlMode
import iiwa_msgs.msg

class mhyServiceState(ServiceState):
    def __init__(self, service_name, service_spec,
            # Request Policy
            request = None,
            request_cb = None,
            request_cb_args = [],
            request_cb_kwargs = {},
            request_key = None,
            request_slots = [],
            # Response Policy
            response_cb = None,
            response_cb_args = [],
            response_cb_kwargs = {},
            response_key = None,
            response_slots = [],
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = [],
            ):
        # ServiceState.__init__(
        #     self，serviceName, serviceType, outcomes=['succeeded', 'aborted','preempted', 'lever','button'])
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted', 'lever', 'button', 'screen'])
        # Store Service info
        self._service_name = service_name
        self._service_spec = service_spec

        self._proxy = None

        # Store request policy
        if request is None:
            self._request = service_spec._request_class()
        else:
            self._request = request

        
        if request_cb is not None and not hasattr(request_cb, '__call__'):
            raise smach.InvalidStateError("Request callback object given to ServiceState that IS NOT a function object")

        self._request_cb = request_cb
        self._request_cb_args = request_cb_args
        self._request_cb_kwargs = request_cb_kwargs
        if smach.has_smach_interface(request_cb):
            self._request_cb_input_keys = request_cb.get_registered_input_keys()
            self._request_cb_output_keys = request_cb.get_registered_output_keys()

            self.register_input_keys(self._request_cb_input_keys)
            self.register_output_keys(self._request_cb_output_keys)
        else:
            self._request_cb_input_keys = input_keys
            self._request_cb_output_keys = output_keys

        self._request_key = request_key
        if request_key is not None:
            self.register_input_keys([request_key])

        self._request_slots = request_slots
        self.register_input_keys(request_slots)

        # Store response policy
        if response_cb is not None and not hasattr(response_cb, '__call__'):
            raise smach.InvalidStateError("Response callback object given to ServiceState that IS NOT a function object")

        self._response_cb = response_cb
        self._response_cb_args = response_cb_args
        self._response_cb_kwargs = response_cb_kwargs
        if smach.has_smach_interface(response_cb):
            self._response_cb_input_keys = response_cb.get_registered_input_keys()
            self._response_cb_output_keys = response_cb.get_registered_output_keys()
            self._response_cb_outcomes = response_cb.get_registered_outcomes()

            self.register_input_keys(self._response_cb_input_keys)
            self.register_output_keys(self._response_cb_output_keys)
            self.register_outcomes(self._response_cb_outcomes)
        else:
            self._response_cb_input_keys = input_keys
            self._response_cb_output_keys = output_keys
            self._response_cb_outcomes = outcomes

        # Register additional input and output keys
        self.register_input_keys(input_keys)
        self.register_output_keys(output_keys)
        self.register_outcomes(outcomes)

        self._response_key = response_key
        if response_key is not None:
            self.register_output_keys([response_key])

        self._response_slots = response_slots
        self.register_output_keys(response_slots)

    def execute(self, ud):
        return super(mhyServiceState, self).execute(ud)

def main():
    rospy.init_node('hangdian_state_machine_1')
    test = True

    # Construct static goals
    # polygon_big = turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0)
    # polygon_small = turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) 
    openGripper = gripper_msgs.srv.gripper_controlRequest()
    openGripper.type = 1
    openGripper.value = 1000

    # 应该从其他状态中得到
    rotateGripper = gripper_msgs.srv.gripper_controlRequest()
    rotateGripper.type = 4
    rotateGripper.value = 90

    targetPose = iiwa_msgs.msg.CartesianPose()
    targetPose.poseStamped.header.frame_id = "iiwa_link_0"
    targetPose.poseStamped.pose.position.x = 
    targetPose.poseStamped.pose.position.y = 
    targetPose.poseStamped.pose.position.z = 

    targetPose.poseStamped.pose.orientation.x = 
    targetPose.poseStamped.pose.orientation.y = 
    targetPose.poseStamped.pose.orientation.z = 
    targetPose.poseStamped.pose.orientation.w = 

    targetPose.redundancy.status = 
    targetPose.redundancy.turn = 
    targetPose.redundancy.e1 = 
    

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


        # 0. 自启动


        # 1. 自检
        # check if sensors are OK
        StateMachine.add('CHECK',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'TASK_MANAGER', 'aborted':'aborted'})


        # 2. 任务管理器
        ## 如果任务未完成，继续检测
        ## 任务完成，则到总sm的succeed状态
        # check if sensors are OK
        if test:
            StateMachine.add('TASK_MANAGER',
                mhyServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION'},)
        else:
            StateMachine.add('TASK_MANAGER',
                mhyServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger, 
                    response_slots=['index_of_button']),
                transitions = {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION', 'screen':'IMAGE_DETECT'},
                remapping = {'index_of_button':'ud_index_of_button'})
        # smach_ros里的ServiceState只有succeeded、aborted、preempted三种状态

        # 3. 操纵杆检测
        # check if sensors are OK
        if test:
            StateMachine.add('DETECT_LEVER',
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                {'succeeded':'IMAGE_DETECT'}) # succeeded should be TASK_M in real
        else:
            # to be modified to Action by lzq
            StateMachine.add('DETECT_LEVER',
                ServiceState('detect_lever', std_srvs.srv.Trigger, input_keys=['lever_input']),
                transitions = {'succeeded':'IMAGE_DETECT'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_index_of_button'})


        # 4. 视觉识别（嵌套状态机）
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with button_detect:
            ## 4.1 到观测位置
            # arm approach panel
            if test:
                StateMachine.add('OBSERVE_POSITION',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'YOLOICP'})
            else:
                # to be modified to Action by mhy
                StateMachine.add('OBSERVE_POSITION',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        request_slots = ['observe_button_input']),
                    transitions = {'succeeded':'YOLOICP'},
                    remapping = {'observe_button_input':'ud_index_of_button'})
            ## 4.2 YOLO-ICP
            # yolo icp for button localization
            if test:
                StateMachine.add('YOLOICP',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'SVM'})
            else:
                StateMachine.add('YOLOICP',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        request_slots = ['yoloicp_input_list'],
                        response_slots=['yoloicp_output_pose']),
                    transitions = {'succeeded':'SVM'},
                    remapping = {'yoloicp_input_list':'ud_index_of_button', 'yoloicp_output_pose':'ud_pose'})

            ## 4.3 状态识别（也许可以和4.2并行？）
            if test:
                StateMachine.add('SVM',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'succeeded'})
            else:
                StateMachine.add('SVM',
                    ServiceState('/svm', std_srvs.srv.Trigger,
                        request_slots = ['svm_input_list'],
                        response_slots=['svm_output_status']),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'svm_input_list':'ud_index_of_button', 'svm_output_status':'ud_status'})
        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})
        
        # 5. 机械臂检测(大action)
        if test:
            StateMachine.add('ARM_DETECT', 
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger), 
                transitions = {'succeeded':'succeeded', 'aborted':'ARM_DETECT', 'preempted':'ARM_DETECT'})# succeeded should be TASK_M in real
        else:
            StateMachine.add('ARM_DETECT', 
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        request_slots = ['arm_input_pose', 'arm_input_status']), 
                transitions = {'succeeded':'succeeded', 'aborted':'ARM_DETECT', 'preempted':'ARM_DETECT'},# succeeded should be TASK_M in real
                remapping = {'arm_input_pose':'ud_index_of_button', 'arm_input_status':'ud_status'})
        
        # 6. 图像识别（嵌套状态机，操作后即可并行）
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with image_detect:
            ## 6.1 到观测位置（如果需要）
            # arm go to observe position
            if test:
                StateMachine.add('OBSERVE_IMAGE',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        ),
                    {'succeeded':'DETECT_IMAGE'})
            else:
                StateMachine.add('OBSERVE_IMAGE',
                    ServiceState('iiwa/move_to_cartesian_pose_lin', std_srvs.srv.Trigger,
                        request_slots = ['observe_image_input']),
                    transitions = {'succeeded':'DETECT_IMAGE'},
                    remapping = {'observe_image_input':'ud_index_of_button'})

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理器
            # arm go to observe position
            if test:
                StateMachine.add('DETECT_IMAGE',
                    ServiceState('linedetection', std_srvs.srv.Trigger),
                    {'succeeded':'succeeded', 'aborted':'DETECT_IMAGE', 'preempted':'DETECT_IMAGE'})
            else:
                StateMachine.add('DETECT_IMAGE',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        request_slots = ['screen_input_list'],
                        response_slots = ['screen_output']),
                    transitions = {'succeeded':'succeeded', 'aborted':'DETECT_IMAGE', 'preempted':'DETECT_IMAGE'},
                    remapping = {'yoloicp_input_list':'ud_index_of_button', 'screen_output':'ud_screen_output'})
        StateMachine.add('IMAGE_DETECT', image_detect, {'succeeded':'succeeded'})

        


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
