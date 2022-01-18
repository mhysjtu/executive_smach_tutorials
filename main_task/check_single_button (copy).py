#!/usr/bin/env python2
#-*- coding: UTF-8 -*-
"""
Description:
    check before acceptance Item 1: single button

Usage:
    $> 
    $> 
"""

from re import U
from time import sleep, time
from actionlib.action_client import TerminalState
import rospy
import actionlib
import threading
import random
from rospy.core import rospyinfo, rospywarn

import smach
from smach import StateMachine, State
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
from geometry_msgs.msg import Pose
from std_msgs.msg import String
# 需要source才能找到�?定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse
from hangdian_msgs.srv import ScreenDetect, ScreenDetectRequest, TaskManage, TaskManageResponse, LocalTable, LocalTableRequest
from hangdian_msgs.msg import LeverDetectAction, LeverDetectGoal, ButtonManipulateAction, ButtonManipulateGoal
from hangdian_msgs.msg import CentralControlRunAction, CentralControlRunResult, CentralControlRunFeedback
from hangdian_msgs.msg import ButtonObserveMoveAction, ButtonObserveMoveGoal

#std_srvs.srv.SetBoolRequest
# from iiwa_msgs.msg import ControlMode
# from iiwa_msgs.srv import ConfigureControlMode
# import iiwa_msgs.msg

central_control_action_result = False

process_info = rospy.Publisher('/hangdian/centralControlInfo', String, queue_size=10)

class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_xml_loaded', 'lever', 'aborted', 'button', 'button_with_state', 'screen'],
                             output_keys=['operation_info', 'operate_type', 'panel', 'objects'])
        self.outcome = 'succeeded'
    def execute(self, userdata):
        self.outcome= 'succeeded'
        rospy.wait_for_service('/hangdian/task_manager_service')
        try:
            getTask = rospy.ServiceProxy('/hangdian/task_manager_service', TaskManage)
            taskResponse = getTask(" ") #人机界面已经请求过xml文件，之后就都发空的service req
            loaded_xml = taskResponse.load_xml
            if loaded_xml:
                have_task = taskResponse.have_task
                if not have_task:
                    print("All task have been completed !!!")
                    self.outcome = 'succeeded'
                    global central_control_action_result
                    central_control_action_result = True
                    actionFeedback.smach_info = 'task finished;'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    return self.outcome
                else:
                    #最后会是一个函数，输入service返回的response——字符串，得到操作种类、面板序号、开关种类、开关序号：
                    print(taskResponse.module)
                    # userdata.operation_info = [taskResponse.panel, taskResponse.operate_type, taskResponse.objects]
                    # # userdata.module = taskResponse.module
                    userdata.operate_type = taskResponse.operate_type
                    userdata.panel = taskResponse.panel
                    userdata.objects = taskResponse.objects

                    actionFeedback.smach_info = 'task manager;'+str(taskResponse.module)+';'+str(taskResponse.operate_type)+';'+str(taskResponse.panel)+';'+str(taskResponse.objects)+';'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    
                    if taskResponse.module == 1:
                        print("To detect button !!!")
                        self.outcome = 'button'
                        # comment for no state detection
                        # if taskResponse.operate_type == 2 or taskResponse.operate_type == 6 or taskResponse.operate_type == 5: # switch button
                        #     userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                        #         int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                        #     self.outcome = 'button_with_state'
                        if taskResponse.operate_type == 2 or taskResponse.operate_type == 6 or taskResponse.operate_type == 5: # switch button
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                            self.outcome = 'button'
                        elif taskResponse.operate_type == 4: # continuous press
                            a = [int(taskResponse.panel), int(taskResponse.operate_type)]
                            b = [int(x) for x in taskResponse.objects]
                            userdata.operation_info = a+b # 第一个元素是数字个数
                            self.outcome = 'button'
                        else:
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), 0]
                            self.outcome = 'button'
                    elif taskResponse.module == 2:
                        print("To detect lever !!!")
                        self.outcome = 'lever'
                        userdata.operation_info = [int(taskResponse.operate_type),taskResponse.panel,
                            int(taskResponse.objects[0])]
                    elif taskResponse.module == 3:
                        # sleep(1.0)
                        print("To detect image !!!")
                        a = [int(taskResponse.operate_type), int(taskResponse.panel)]
                        b = [x for x in taskResponse.objects]
                        userdata.operation_info = a + b
                        self.outcome = 'screen'
                        # print(userdata.operation_info)
                        #if userdata.operate_type == 4:
                        #   userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type)]
                        #   for i in taskResponse.objects
                        #       userdata.operation_info.push(int(i))
            else:
                self.outcome = 'no_xml_loaded'
            return self.outcome
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.outcome = 'aborted'
            return self.outcome

def execute(goal):
    global central_control_action_result
    
    test = False
    useICP = False

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    # sm0.userdata.final_result = None
    # Open the container
    with sm0:
        # 0. �?�?�?
        global actionFeedback
        actionFeedback = CentralControlRunFeedback()
        global actionResult
        actionResult = CentralControlRunResult()
        actionResult.result = True

        # 1. �?检
        # check if sensors are OK
        @smach.cb_interface(output_keys=['current_pose'])
        def check_response_cb(userdata, response):
            userdata.current_pose = 11
            return
        StateMachine.add('CHECK',
            ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                response_cb=check_response_cb),
            transitions = {'succeeded':'TASK_MANAGER', 'aborted':'aborted'},
            remapping = {'current_pose':'ud_current_pose'})
        actionFeedback.smach_info = 'self check'
        actionServer.publish_feedback(actionFeedback)
        process_info.publish(actionFeedback.smach_info)

        # 2. 任务管理�?
        ## 如果任务�?完成，继�?检�?
        ## 任务完成，则到总sm的succeed状�?
        # check if sensors are OK
        if test:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'succeeded', 'no_xml_loaded':'TASK_MANAGER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION', 'button_with_state':'RECOGNITION_WITH_SVM'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
            
        else:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'RETURN_TO_ORIGIN', 'no_xml_loaded':'TASK_MANAGER', 'lever':'DETECT_LEVER', 'button':'RECOGNITION', 'button_with_state':'RECOGNITION_WITH_SVM',
                               'screen':'DETECT_SCREEN'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
        # smach_ros里的ServiceState�?有succeeded、aborted、preempted三�?�状�?

        if test:
            @smach.cb_interface(input_keys=['lever_input'], output_keys=['result'], outcomes=['succeeded'])
            def lever_response_cb(userdata, response):
                if userdata.lever_input == '1':
                    userdata.result = True
                else:
                    userdata.result = False
                # status = userdata.result # 除了不能print output_keys, �?的也没问题了
                # print userdata.final_result
                actionFeedback.smach_info = 'lever detect'
                actionServer.publish_feedback(actionFeedback)
                return
            StateMachine.add('DETECT_LEVER',
                ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                    response_cb=lever_response_cb),
                transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_operate_type'} # userdata in viewer are only ud_xxx and final_xxx
                )
        else:
            @smach.cb_interface(input_keys=['lever_input'])
            def lever_goal_cb(userdata, goal):  
                actionFeedback.smach_info = 'lever detect started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                lever_goal = LeverDetectGoal()
                print(userdata.lever_input[0])
                lever_goal.handle_command = str(userdata.lever_input[0])+'-'+str(userdata.lever_input[1])+'-'+str(userdata.lever_input[2])
                return lever_goal
            def lever_result_cb(userdata, status, result): 
                actionFeedback.smach_info = 'lever detect finished'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                return 'succeeded'
            StateMachine.add('DETECT_LEVER',
                SimpleActionState('air_handle', LeverDetectAction,
                    goal_cb=lever_goal_cb,
                    result_cb=lever_result_cb),
                transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_operation_info'})

        # 4. 视�?�识�?（嵌套状态机�?
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose'])
        with button_detect:
            ## 4.1 到�?�测位置
            # arm approach panel
            if test:
                def observe_response_cb(userdata, response):
                    actionFeedback.smach_info = 'go to observation position'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('OBSERVE_POSITION',
                    ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                        response_slots=['message'],
                        response_cb=observe_response_cb),
                    transitions = {'succeeded':'YOLOICP'})
            else:
                ####### to be modified to Action by mhy ###### 
                # StateMachine.add('OBSERVE_POSITION',
                #     SimpleActionState('large_move_server', std_srvs.srv.Trigger,
                #         goal_slots = ['button_index']),
                #     transitions = {'succeeded':'YOLOICP'},
                #     remapping = {'button_index':'ud_operation_info'})
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    actionFeedback.smach_info = 'go to observe pose started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    observe_goal = ButtonObserveMoveGoal()
                    print(userdata.observe_input[0])
                    observe_goal.button_index = [userdata.current_pose, userdata.observe_input[0]]
                    return observe_goal
                @smach.cb_interface(input_keys=['observe_input'], output_keys=['current_pose'])
                def observe_result_cb(userdata, status, result): 
                    # sleep(1.0)
                    userdata.current_pose = userdata.observe_input[0]
                    actionFeedback.smach_info = 'observe pose reached'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    return 'succeeded'
                StateMachine.add('OBSERVE_POSITION',
                    SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'YOLOICP'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose'})


            ## 4.2 YOLO-ICP
            # yolo icp for button localization
            if test:
                def yolo_response_cb(userdata, response):
                    actionFeedback.smach_info = 'yolo detect'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('YOLOICP',
                    ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                        response_slots=['message'],
                        response_cb=yolo_response_cb),
                    transitions={'succeeded':'SVM'},
                    remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})
            elif useICP:
                @smach.cb_interface(input_keys=['icp_input'])
                def icp_request_cb(userdata, request):
                    actionFeedback.smach_info = 'icp detect started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    icp_request = ButtonDetectPoseRequest()
                    icp_request.button_index = int(userdata.icp_input[0])
                    print(icp_request.button_index)
                    return icp_request
                @smach.cb_interface(output_keys=['state'])
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    print("yolo result is: ")
                    print(response.pose_in_camera)
                    userdata.state = [0, 0]
                    return 
                StateMachine.add('YOLOICP',
                    ServiceState('/yolo_icp_pose', ButtonDetectPose,
                        request_cb=icp_request_cb,
                        response_slots=['pose_in_camera'],
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose', 'state':'ud_status'})
                # def yolo_response_cb(userdata, response):
                #     actionFeedback.smach_info = 'yolo detect'
                #     actionServer.publish_feedback(actionFeedback)
                #     if response.success == True:
                #         return
                #     else:
                #         return
                # StateMachine.add('YOLOICP',
                #     ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                #         response_slots=['message'],
                #         response_cb=yolo_response_cb),
                #     transitions={'succeeded':'SVM'},
                #     remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})
            else:
                # 不用icp定位，获取已知的定位结果
                @smach.cb_interface(input_keys=['icp_input'])
                def icp_request_cb(userdata, request):
                    actionFeedback.smach_info = 'icp detect started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    table_request = LocalTableRequest()
                    table_request.table_type = 5
                    table_request.panel_index = int(userdata.icp_input[0])
                    print(table_request.panel_index)
                    return table_request
                @smach.cb_interface(input_keys=['icp_input'], output_keys=['state', 'pose_in_camera'])
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    print("localization table result is: ")
                    print(response.pose_in_camera)
                    # input('input to continue')
                    userdata.pose_in_camera = response.pose_in_camera
                    userdata.state = [0, int(userdata.icp_input[3])]
                    return
                StateMachine.add('YOLOICP',
                    ServiceState('/hangdian/info/local_tables', LocalTable,
                        request_cb=icp_request_cb,
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose', 'state':'ud_status'})


        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})


        button_detect_with_svm = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose'])
        with button_detect_with_svm:
            ## 4.1 到�?�测位置
            # arm approach panel
            if test:
                def observe_response_cb(userdata, response):
                    actionFeedback.smach_info = 'go to observation position'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('OBSERVE_POSITION',
                    ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                        response_slots=['message'],
                        response_cb=observe_response_cb),
                    transitions = {'succeeded':'YOLOICP'})
            else:
                ####### to be modified to Action by mhy ###### 
                # StateMachine.add('OBSERVE_POSITION',
                #     SimpleActionState('large_move_server', std_srvs.srv.Trigger,
                #         goal_slots = ['button_index']),
                #     transitions = {'succeeded':'YOLOICP'},
                #     remapping = {'button_index':'ud_operation_info'})
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    actionFeedback.smach_info = 'go to observe pose started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    observe_goal = ButtonObserveMoveGoal()
                    print(userdata.observe_input[0])
                    observe_goal.button_index = [userdata.current_pose, userdata.observe_input[0]]
                    return observe_goal
                @smach.cb_interface(input_keys=['observe_input'], output_keys=['current_pose'])
                def observe_result_cb(userdata, status, result): 
                    # sleep(1.0)
                    userdata.current_pose = userdata.observe_input[0]
                    actionFeedback.smach_info = 'observe pose reached'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    return 'succeeded'
                StateMachine.add('OBSERVE_POSITION',
                    SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'YOLOICP'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose'})


            ## 4.2 YOLO-ICP
            # yolo icp for button localization
            if test:
                def yolo_response_cb(userdata, response):
                    actionFeedback.smach_info = 'yolo detect'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('YOLOICP',
                    ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                        response_slots=['message'],
                        response_cb=yolo_response_cb),
                    transitions={'succeeded':'SVM'},
                    remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})
            else:
                @smach.cb_interface(input_keys=['icp_input'])
                def icp_request_cb(userdata, request):
                    actionFeedback.smach_info = 'icp detect started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    icp_request = ButtonDetectPoseRequest()
                    icp_request.button_index = int(userdata.icp_input[0])
                    print(icp_request.button_index)
                    return icp_request
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    print("yolo result is: ")
                    print(response.pose_in_camera)
                    return 
                StateMachine.add('YOLOICP',
                    ServiceState('/yolo_icp_pose', ButtonDetectPose,
                        request_cb=icp_request_cb,
                        response_slots=['pose_in_camera'],
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'SVM'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose'})
                # def yolo_response_cb(userdata, response):
                #     actionFeedback.smach_info = 'yolo detect'
                #     actionServer.publish_feedback(actionFeedback)
                #     if response.success == True:
                #         return
                #     else:
                #         return
                # StateMachine.add('YOLOICP',
                #     ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                #         response_slots=['message'],
                #         response_cb=yolo_response_cb),
                #     transitions={'succeeded':'SVM'},
                #     remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})

            ## 4.3 状态识别（也许可以和4.2并�?�？�?
            if test:
                def svm_response_cb(userdata, response):
                    actionFeedback.smach_info = 'svm detect'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('SVM',
                    ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                        response_slots=['message'],
                        response_cb=svm_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'button_index':'ud_operation_info', 'message':'ud_status'})
            else:
                @smach.cb_interface(input_keys=['svm_input'])
                def svm_request_cb(userdata, request):
                    actionFeedback.smach_info = 'svm detect started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    svm_request = ButtonDetectStateRequest()
                    svm_request.panel_index = int(userdata.svm_input[0])
                    svm_request.button_index = userdata.svm_input[2]
                    return svm_request
                @smach.cb_interface(input_keys=['svm_input'], output_keys=['states'])
                def svm_response_cb(userdata, response):
                    actionFeedback.smach_info = 'svm detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    print("svm result is: ")
                    print(response.state)
                    userdata.states = [response.state, userdata.svm_input[3]]
                    return
                StateMachine.add('SVM',
                    ServiceState('/svm', ButtonDetectState,
                        request_cb=svm_request_cb,
                        response_cb=svm_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'svm_input':'ud_operation_info', 'states':'ud_status'})
        StateMachine.add('RECOGNITION_WITH_SVM',button_detect_with_svm, {'succeeded':'ARM_DETECT'})
        
        # 5. 机械臂检测(用action)
        if test:
            def arm_response_cb(userdata, response):
                actionFeedback.smach_info = 'arm detect'
                actionServer.publish_feedback(actionFeedback)
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('ARM_DETECT', 
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                    response_cb=arm_response_cb), 
                transitions = {'succeeded':'TASK_MANAGER'})# succeeded should be TASK_M in real
            # StateMachine.add('ARM_DETECT', 
            #     SimpleActionState('detect_with_arm_gripper', ButtonManipulateAction, goal = manipulateGoal), 
            #     transitions = {'succeeded':'succeeded'})
            
        else:
            ####### to be modified to Action by mhy ######
            StateMachine.add('ARM_DETECT', 
                SimpleActionState('manipulate_controller_iiwa2', ButtonManipulateAction, 
                    goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
                transitions = {'succeeded':'TASK_MANAGER'},
                remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
            # def arm_response_cb(userdata, response):
            #     actionFeedback.smach_info = 'arm detect'
            #     actionServer.publish_feedback(actionFeedback)
            #     if response.success == True:
            #         return
            #     else:
            #         return
            # StateMachine.add('ARM_DETECT', 
            #     ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
            #         response_cb=arm_response_cb), 
            #     transitions = {'succeeded':'TASK_MANAGER'})
        

        # 6. 图像识别（嵌套状态机，操作后即可并�?�）
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info', 'ud_panel'])
        with image_detect:
            ## 6.1 到�?�测位置（�?�果需要）
            # arm go to observe position
            if test:
                def ob_image_response_cb(userdata, response):
                    actionFeedback.smach_info = 'go to observation position'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('OBSERVE_IMAGE',
                    ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                        response_cb=ob_image_response_cb),
                    {'succeeded':'DETECT_IMAGE'})
            else:
                ####### to be modified to Action by mhy ######
                # 需要判�?�?否需要到观测位置，�?�果不用就不调用largemove action
                # StateMachine.add('OBSERVE_IMAGE',
                #     SimpleActionState('large_move_server', std_srvs.srv.Trigger,
                #         goal_slots = ['button_index']),
                #     transitions = {'succeeded':'DETECT_SCREEN'},
                #     remapping = {'button_index':'ud_index_of_button'})
                def ob_image_response_cb(userdata, response):
                    actionFeedback.smach_info = 'go to observation position'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('OBSERVE_IMAGE',
                    ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                        response_cb=ob_image_response_cb),
                    {'succeeded':'DETECT_IMAGE'})

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理�?
            # arm go to observe position
            if test:
                def image_response_cb(userdata, response):
                    actionFeedback.smach_info = 'image detect'
                    actionServer.publish_feedback(actionFeedback)
                    if response.success == True:
                        return
                    else:
                        return
                StateMachine.add('DETECT_IMAGE',
                    ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                        response_cb=image_response_cb),
                    {'succeeded':'succeeded'})
            else:
                @smach.cb_interface(input_keys=['image_input'])
                def image_request_cb(userdata, request):
                    actionFeedback.smach_info = 'image detect'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    image_request = ScreenDetectRequest()
                    image_request.param_b = int(userdata.image_input[0])#b
                    image_request.param_c = int(userdata.image_input[1])#c
                    image_request.param_d = userdata.image_input[2]#d
                    image_request.param_e = int(userdata.image_input[3])#e
                    image_request.param_f = int(userdata.image_input[4])#f
                    image_request.param_g = int(userdata.image_input[5])#g
                    # print(image_request.param_b+image_request.param_c+int(image_request.param_d)+image_request.param_e+image_request.param_f+image_request.param_g)
                    return image_request
                def image_response_cb(userdata, response):
                    actionFeedback.smach_info = 'image detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    print("image detection result is: ")
                    print(response.success)
                    rospy.logwarn("image detection result is: ")
                    rospy.logwarn(response.success)
                    return
                
                StateMachine.add('DETECT_IMAGE',
                    ServiceState('/halcon_detection_service', ScreenDetect,
                        request_cb=image_request_cb,
                        response_cb=image_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'image_input':'ud_operation_info'})
        StateMachine.add('DETECT_SCREEN', image_detect, {'succeeded':'TASK_MANAGER'})


        @smach.cb_interface(input_keys=['current_pose'])
        def observe_goal_cb(userdata, goal):  
            actionFeedback.smach_info = 'go to origin pose 11 started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            observe_goal = ButtonObserveMoveGoal()
            observe_goal.button_index = [userdata.current_pose, 11]
            return observe_goal
        @smach.cb_interface(output_keys=['current_pose'])
        def observe_result_cb(userdata, status, result): 
            # sleep(1.0)
            userdata.current_pose = 11
            actionFeedback.smach_info = 'origin pose reached'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            return 'succeeded'
        StateMachine.add('RETURN_TO_ORIGIN',
            SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                goal_cb=observe_goal_cb,
                result_cb=observe_result_cb),
            transitions = {'succeeded':'succeeded'},# succeeded should be TASK_M in real
            remapping = {'current_pose':'ud_current_pose'})

        

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_demo_1', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    while not central_control_action_result:
        sleep(0.1)
    actionServer.set_succeeded(actionResult)
    

def main():
    rospy.init_node('hangdian_state_machine_test_with_GUI')
    global actionServer
    actionServer = actionlib.SimpleActionServer('central_control', CentralControlRunAction, execute, auto_start=False)
    actionServer.start()
    rospy.loginfo('central control server started')

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
