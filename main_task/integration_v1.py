#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
"""
Description:
Usage:
    $> cd ~/aircraft_ws/src/hangdian_detection/hangdian_arm_controllers/gripper_controllers/scripts
    $> python empty_service_node.py
    in this directory:
        $> python SM_1.py
        $> rosrun smach_viewer smach_viewer.py
"""

from numpy.core.numeric import require
import rospy
import threading
import random

import smach
from smach import StateMachine, State
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
from geometry_msgs.msg import Pose
# 需要source才能找到自定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse, ScreenDetect, ScreenDetectRequest
from hangdian_msgs.msg import LeverDetectAction, LeverDetectGoal, ButtonManipulateAction, ButtonManipulateGoal

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

def easyTaskResolver(message):
    #message="1"-"3"
    # 面板检测
    if message=='1':
        op_info = '1'
    # 操纵杆检测
    elif message=='2':
        op_info = 'JSA'
    # 图像检测
    elif message=='3':
        index = random.randint(1,3)
        if index==1:
            op_info = '18'
        elif index==2:
            op_info = '19'
        elif index ==3:
            op_info = '17'
    return op_info


class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'lever', 'button', 'screen'], output_keys=['operation_info'])
        self.outcome = 'succeeded'
        # self.index_of_button = None
    def execute(self, userdata):
        self.outcome= 'succeeded'
        rospy.wait_for_service('hangdian/smach/empty_service')
        try:
            get_button_index = rospy.ServiceProxy('hangdian/smach/empty_service', std_srvs.srv.Trigger)
            response = get_button_index()
            output = easyTaskResolver(response.message)
            
            #最后会是一个函数，输入service返回的response——字符串，得到操作种类、面板序号、开关种类、开关序号：
            userdata.operation_info = output

            if response.message =='1':
                print("detect button !!!")
                self.outcome = 'button'
            elif response.message =='2':
                print("detect lever !!!")
                self.outcome = 'lever'
            elif response.message =='3':
                print("detect screen !!!")
                self.outcome = 'screen'
            return self.outcome
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.outcome = 'aborted'
            return self.outcome

def main():
    rospy.init_node('hangdian_state_machine_1')

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    # sm0.userdata.final_result = None
    # Open the container
    with sm0:
        # 0. 自启动


        # 1. 自检
        # check if sensors are OK
        StateMachine.add('CHECK',
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger),
                {'succeeded':'TASK_MANAGER', 'aborted':'aborted'})


        # 2. 任务管理器
        ## 如果任务未完成，继续检测
        ## 任务完成，则到总sm的succeed状态
        # check if sensors are OK

        StateMachine.add('TASK_MANAGER',
            TaskServiceCB(),
            transitions = {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
            remapping = {'operation_info':'ud_operation_info'})
        # else:
        #     StateMachine.add('TASK_MANAGER',
        #         TaskServiceCB(),
        #         transitions = {'succeeded':'DETECT_LEVER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
        #         remapping = {'index_of_button':'ud_index_of_button'})
        # smach_ros里的ServiceState只有succeeded、aborted、preempted三种状态


        # 3. 操纵杆检测
        # check if sensors are OK

        # lever_req = std_srvs.srv.SetBoolRequest()
        # lever_req.data = True
        # @smach.cb_interface(input_keys=['lever_input'], output_keys=['result'], outcomes=['succeeded'])
        # def lever_response_cb(userdata, response):
        #     if userdata.lever_input == 'lever':
        #         userdata.result = True
        #     else:
        #         userdata.result = False
        #     # status = userdata.result # 除了不能print output_keys, 别的也没问题了
        #     # print userdata.final_result
        #     return
        # StateMachine.add('DETECT_LEVER',
        #     ServiceState('hangdian/smach/empty_service_req', std_srvs.srv.SetBool,
        #         request = lever_req,
        #         response_cb=lever_response_cb),
        #     transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
        #     remapping = {'lever_input':'ud_index_of_button', 'result':'final_result'} # userdata in viewer are only ud_xxx and final_xxx
        #     )
        StateMachine.add('DETECT_LEVER',
            SimpleActionState('air_handle', LeverDetectAction,
                goal_slots = ['handle_command']),
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'handle_command':'ud_operation_info'})
        # else:
        #     StateMachine.add('DETECT_LEVER',
        #         SimpleActionState('airhandle_server', LeverDetectAction,
        #             goal_slots = ['handle_command']),
        #         transitions = {'succeeded':'RECOGNITION'},# succeeded should be TASK_M in real
        #         remapping = {'handle_command':'ud_index_of_button'})


        # 4. 视觉识别（嵌套状态机）
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info'], output_keys=['ud_pose', 'ud_status'])
        with button_detect:
            ## 4.1 到观测位置
            # arm approach panel

            def observe_response_cb(userdata, response):
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('OBSERVE_POSITION',
                ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                    response_slots=['message'],
                    response_cb=observe_response_cb),
                transitions = {'succeeded':'YOLOICP'})
            # else:
            #     ####### to be modified to Action by mhy ###### 
            #     StateMachine.add('OBSERVE_POSITION',
            #         SimpleActionState('large_move_server', std_srvs.srv.Trigger,
            #             goal_slots = ['button_index']),
            #         transitions = {'succeeded':'YOLOICP'},
            #         remapping = {'button_index':'ud_index_of_button'})

            ## 4.2 YOLO-ICP
            # yolo icp for button localization
            def yolo_response_callback(userdata, response):
                print("yolo result is: ")
                print(response.pose_in_camera)
                return 
            StateMachine.add('YOLOICP',
                ServiceState('/yolo_icp_pose', ButtonDetectPose,
                    request_slots = ['button_index'],
                    response_slots=['pose_in_camera'],
                    response_cb=yolo_response_callback),
                transitions={'succeeded':'SVM'},
                remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose'})            


            ## 4.3 状态识别（也许可以和4.2并行？）
            def svm_response_callback(userdata, response):
                print("svm result is: ")
                print(response.state)
                return 
            StateMachine.add('SVM',
                ServiceState('/svm', ButtonDetectState,
                    request_slots = ['button_index'],
                    response_slots=['state'],
                    response_cb=svm_response_callback),
                transitions = {'succeeded':'succeeded'},
                remapping = {'button_index':'ud_operation_info', 'state':'ud_status'})
            # else:
            #     StateMachine.add('SVM',
            #         ServiceState('/svm', ButtonDetectState,
            #             request_slots = ['button_index'],
            #             response_slots=['state']),
            #         transitions = {'succeeded':'succeeded'},
            #         remapping = {'button_index':'ud_index_of_button', 'state':'ud_status'})
        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})
        
        poseInCamera = Pose()
        poseInCamera.position.x = 0.0
        poseInCamera.position.y = 0.0
        poseInCamera.position.z = 0.25
        poseInCamera.orientation.w = 1.0
        poseInCamera.orientation.x = 0.0
        poseInCamera.orientation.y = 0.0
        poseInCamera.orientation.z = 0.0
        
        manipulateGoal = ButtonManipulateGoal(button_index = "1", pose_in_camera = poseInCamera, button_state = 2)
        # 5. 机械臂检测(大action)
        # StateMachine.add('ARM_DETECT', 
        #     SimpleActionState('detect_with_arm_gripper', ButtonManipulateAction, goal = manipulateGoal), 
        #     transitions = {'succeeded':'TASK_MANAGER'})
        StateMachine.add('ARM_DETECT', 
            SimpleActionState('detect_with_arm_gripper', ButtonManipulateAction, goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
        

        # 6. 图像识别（嵌套状态机，操作后即可并行）
        #嵌套的状态机要传递userdata的话，需要设置input_keys
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info'])
        with image_detect:
            ## 6.1 到观测位置（如果需要）
            # arm go to observe position

            def ob_image_response_cb(userdata, response):
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('OBSERVE_IMAGE',
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                    response_cb=ob_image_response_cb),
                {'succeeded':'DETECT_IMAGE'})
            # else:
            #     ####### to be modified to Action by mhy ######
            #     # 需要判断是否需要到观测位置，如果不用就不调用largemove action
            #     StateMachine.add('OBSERVE_IMAGE',
            #         SimpleActionState('large_move_server', std_srvs.srv.Trigger,
            #             goal_slots = ['button_index']),
            #         transitions = {'succeeded':'DETECT_IMAGE'},
            #         remapping = {'button_index':'ud_index_of_button'})

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理器
            # arm go to observe position
            imageDetect = ScreenDetectRequest()
            imageDetect.category = 19
            # imageDetect.panel_index = None
            # imageDetect.target_index = None
            # imageDetect.context = None

            @smach.cb_interface(input_keys=['image_input'])
            def image_request_cb(userdata, request):
                image_request = ScreenDetectRequest()
                image_request.category = int(userdata.image_input)
                return image_request
            StateMachine.add('DETECT_IMAGE',
                ServiceState('/ScreenDetect', ScreenDetect,
                    request_cb=image_request_cb),
                transitions = {'succeeded':'succeeded'},
                remapping = {'image_input':'ud_operation_info'})
        StateMachine.add('IMAGE_DETECT', image_detect, {'succeeded':'TASK_MANAGER'})

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