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
# 需要source才能找到自定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse

#std_srvs.srv.SetBoolRequest
# from iiwa_msgs.msg import ControlMode
# from iiwa_msgs.srv import ConfigureControlMode
# import iiwa_msgs.msg

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
        State.__init__(self, 
            outcomes=['succeeded', 'aborted', 'preempted', 'lever', 'button', 'screen'])
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

class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'lever', 'button', 'screen'], output_keys=['index_of_button'])
        self.outcome = 'succeeded'
        self.index_of_button = None
    def execute(self, userdata):
        self.outcome= 'succeeded'
        rospy.wait_for_service('hangdian/gripper1/control_service')
        try:
            get_button_index = rospy.ServiceProxy('hangdian/gripper1/control_service', std_srvs.srv.Trigger)
            response = get_button_index()
            userdata.index_of_button = response.message
            if response.message =='button':
                print("detect button !!!")
                self.outcome = 'button'
            elif response.message =='lever':
                print("detect lever !!!")
                self.outcome = 'lever'
            elif response.message =='screen':
                print("detect screen !!!")
                self.outcome = 'screen'
            return self.outcome
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.outcome = 'aborted'
            return self.outcome

def main():
    rospy.init_node('hangdian_state_machine_1')
    test = True

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    # sm0.userdata.final_result = None
    # Open the container
    with sm0:
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
            # def task_response_cb(userdata, response):
            #     # 要用userdata, 必须输入input，和action的result cb不一样
            #     # 如果要输出data可能需要用@smach.cb_interface:
            #         # @smach.cb_interface(input_keys=['pose_index', 'look_poses'], outcomes=['succeeded'])
            #         # def robot_goal_cb(userdata, goal):
            #         #     goal = RobotMoveGoal(userdata.look_poses[userdata.pose_index],False)
            #         #     return goal
    
            #         # @smach.cb_interface(output_keys=['holes_poses'], outcomes=['succeeded'])
            #         # def estimate_response_cb(userdata, response):
            #         #     userdata.holes_poses = response.holes_poses
            #         #     return
            #     if response.message == 'return':
            #         print('111')

            StateMachine.add('TASK_MANAGER',
                # mhyServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                #     response_cb=task_response_cb,
                #     response_slots=['success', 'message']),
                TaskServiceCB(),
                transitions = {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
                remapping = {'index_of_button':'ud_index_of_button'})
        else:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
                remapping = {'index_of_button':'ud_index_of_button'})
        # smach_ros里的ServiceState只有succeeded、aborted、preempted三种状态

        # 3. 操纵杆检测
        # check if sensors are OK
        if test:
            lever_req = std_srvs.srv.SetBoolRequest()
            lever_req.data = True
            @smach.cb_interface(input_keys=['lever_input'], output_keys=['result'], outcomes=['succeeded'])
            def lever_response_cb(userdata, response):
                if userdata.lever_input == 'lever':
                    userdata.result = True
                else:
                    userdata.result = False
                # status = userdata.result # 除了不能print output_keys, 别的也没问题了
                # print userdata.final_result
                return
            StateMachine.add('DETECT_LEVER',
                ServiceState('hangdian/gripper1/control_service_req', std_srvs.srv.SetBool,
                    request = lever_req,
                    response_cb=lever_response_cb),
                transitions = {'succeeded':'IMAGE_DETECT'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_index_of_button', 'result':'final_result'} # userdata in viewer are only ud_xxx and final_xxx
                )
        else:
            # @smach.cb_interface(output_keys=['result'], outcomes=['succeeded'])
            # def lever_response_cb(userdata, response):
            #     userdata.result = response.success
            #     print(userdata.result)
            #     return
            ###### to be modified to Action by lzq ######
            StateMachine.add('DETECT_LEVER',
                ServiceState('detect_lever', std_srvs.srv.Trigger,
                    request_slots = ['button_index']),
                transitions = {'succeeded':'IMAGE_DETECT'},# succeeded should be TASK_M in real
                remapping = {'button_index':'ud_index_of_button'})


        # 4. 视觉识别（嵌套状态机）
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with button_detect:
            ## 4.1 到观测位置
            # arm approach panel
            if test:
                StateMachine.add('OBSERVE_POSITION',
                    ServiceState('hangdian/gripper1/control_service',std_srvs.srv.Trigger,
                        response_slots=['message']),
                    transitions = {'succeeded':'YOLOICP'})
            else:
                ####### to be modified to Action by mhy ###### 
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
                    remapping={'succeeded':'SVM'})
            else:
                StateMachine.add('YOLOICP',
                    ServiceState('/yolo_icp_pose', ButtonDetectPose,
                        request_slots = ['button_index'],
                        response_slots=['pose_in_camera']),
                    transitions = {'succeeded':'SVM'},
                    remapping = {'button_index':'ud_index_of_button', 'pose_in_camera':'ud_pose'})

            ## 4.3 状态识别（也许可以和4.2并行？）
            if test:
                StateMachine.add('SVM',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'succeeded'})
            else:
                StateMachine.add('SVM',
                    ServiceState('/svm', ButtonDetectState,
                        request_slots = ['button_index'],
                        response_slots=['state']),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'button_index':'ud_index_of_button', 'state':'ud_status'})
        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})
        
        # 5. 机械臂检测(大action)
        if test:
            StateMachine.add('ARM_DETECT', 
                ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger), 
                transitions = {'succeeded':'succeeded', 'aborted':'ARM_DETECT', 'preempted':'ARM_DETECT'})# succeeded should be TASK_M in real
        else:
            ####### to be modified to Action by mhy ######
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
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'DETECT_IMAGE'})
            else:
                ####### to be modified to Action by mhy ######
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
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger),
                    {'succeeded':'succeeded', 'aborted':'DETECT_IMAGE', 'preempted':'DETECT_IMAGE'})
            else:
                ####### to be modified to service by npy ######
                StateMachine.add('DETECT_IMAGE',
                    ServiceState('hangdian/gripper1/control_service', std_srvs.srv.Trigger,
                        request_slots = ['screen_input_list'],
                        response_slots = ['screen_output']),
                    transitions = {'succeeded':'succeeded', 'aborted':'DETECT_IMAGE', 'preempted':'DETECT_IMAGE'},
                    remapping = {'yoloicp_input_list':'ud_index_of_button', 'screen_output':'ud_screen_output'})
        StateMachine.add('IMAGE_DETECT', image_detect, {'succeeded':'succeeded'})

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
