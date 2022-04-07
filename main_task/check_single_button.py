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
# 需要source才能找到自定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse
from hangdian_msgs.srv import ScreenDetect, ScreenDetectRequest, ScreenDetectReturn, ScreenDetectReturnRequest, TaskManage, TaskManageResponse, LocalTable, LocalTableRequest
from hangdian_msgs.msg import LeverDetectAction, LeverDetectGoal, ButtonManipulateAction, ButtonManipulateGoal
from hangdian_msgs.msg import CentralControlRunAction, CentralControlRunResult, CentralControlRunFeedback
from hangdian_msgs.msg import ButtonObserveMoveAction, ButtonObserveMoveGoal

#std_srvs.srv.SetBoolRequest
# from iiwa_msgs.msg import ControlMode
# from iiwa_msgs.srv import ConfigureControlMode
# import iiwa_msgs.msg

tic = time()
toc = time()
central_control_action_result = False

number = {0:27, 1:8, 2:9, 3:10, 4:14, 5:15, 6:16, 7:20, 8:21, 9:22 }

process_info = rospy.Publisher('/hangdian/centralControlInfo', String, queue_size=10)

class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_xml_loaded', 'lever', 'aborted', 'button1', 'button2', 'button_with_state1', 'button_with_state2', 'screen', 'screen_return', 'light', 'manual'],
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
                    global toc
                    global tic
                    toc = time()
                    rospy.logwarn("task time")
                    rospy.logwarn(toc-tic)

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
                    
                    global toc
                    global tic
                    toc = time()
                    rospy.logwarn("task time")
                    rospy.logwarn(toc-tic)

                    # 右臂检测
                    if taskResponse.module == 0:
                        print("To detect right button !!!")
                        self.outcome = 'button1'
                        if taskResponse.operate_type == 2:
                            # 面板编号，操作类型，开关编号，开关目标状态
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                            self.outcome = 'button1'
                        elif taskResponse.operate_type == 4: # continuous press
                            a = [int(taskResponse.panel), int(taskResponse.operate_type)]
                            b = [int(x) for x in taskResponse.objects]
                            userdata.operation_info = a+b # 第一个元素是数字个数
                            self.outcome = 'button1'
                        else:
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]),0]
                            self.outcome = 'button1'
                    # 左臂检测
                    elif taskResponse.module == 1:
                        print("To detect left button !!!")
                        self.outcome = 'button2'
                        if taskResponse.operate_type == 2 or taskResponse.operate_type == 3 or taskResponse.operate_type == 6 or taskResponse.operate_type == 5: # switch button
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                            self.outcome = 'button2' # 'button_with_state2'
                        elif taskResponse.operate_type == 4: # continuous press
                            a = [int(taskResponse.panel), int(taskResponse.operate_type)]
                            b = [int(x) for x in taskResponse.objects]
                            userdata.operation_info = a+b # 第一个元素是数字个数
                            self.outcome = 'button2'
                        else:
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]),0]
                            self.outcome = 'button2'
                    # 操纵杆检测
                    elif taskResponse.module == 2:
                        print("To detect lever !!!")
                        self.outcome = 'lever'
                        userdata.operation_info = [int(taskResponse.operate_type),taskResponse.panel,
                            int(taskResponse.objects[0])]
                    # 图像检测 
                    elif taskResponse.module == 3:
                        sleep(0.5)
                        print("To detect image !!!")
                        a = [int(taskResponse.operate_type), int(taskResponse.panel)]
                        b = [x for x in taskResponse.objects]
                        userdata.operation_info = a + b

                        # mhy for mid term exam
                        # self.outcome = 'screen'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam


                        # print(userdata.operation_info)
                        #if userdata.operate_type == 4:
                        #   userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type)]
                        #   for i in taskResponse.objects
                        #       userdata.operation_info.push(int(i))
                    # 灯光检测    
                    elif taskResponse.module == 4:
                        print("To detect light !!!")
                        userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                        print([int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])])
                        # mhy for mid term exam
                        # self.outcome = 'light'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
                    # 图像检测返回结果
                    elif taskResponse.module == 5:
                        sleep(0.5)
                        print("To detect image and return result!!!")
                        a = [int(taskResponse.operate_type), int(taskResponse.panel)]
                        b = [x for x in taskResponse.objects]
                        userdata.operation_info = a + b
                        
                        # mhy for mid term exam
                        # self.outcome = 'screen_return'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
                    elif taskResponse.module == 6:
                        rospy.logwarn("请手动操作：")
                        rospy.logwarn(taskResponse.panel)
                        rospy.logwarn('操作完毕后，请输入3,并按回车继续。。。')
                        input()
                        
                        # mhy for mid term exam
                        # self.outcome = 'manual'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
            else:
                self.outcome = 'no_xml_loaded'
            # rospy.logwarn('input to start this task')
            # input()
            return self.outcome
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.outcome = 'aborted'
            return self.outcome

def execute(goal):
    global central_control_action_result
    
    test = False

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    # sm0.userdata.final_result = None
    # Open the container
    with sm0:
        # 0. action的feedback和result全局变量定义
        global actionFeedback
        actionFeedback = CentralControlRunFeedback()
        global actionResult
        actionResult = CentralControlRunResult()
        actionResult.result = True

        # 1. 机械臂初始面板位置
        @smach.cb_interface(output_keys=['current_pose1', 'current_pose2'])
        def check_response_cb(userdata, response):
            userdata.current_pose1 = 18
            userdata.current_pose2 = 11
            global tic
            tic = time()
            return
        StateMachine.add('CHECK',
            ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                response_cb=check_response_cb),
            transitions = {'succeeded':'TASK_MANAGER', 'aborted':'aborted'},
            remapping = {'current_pose1':'ud_current_pose1', 'current_pose2':'ud_current_pose2'})
        actionFeedback.smach_info = 'self check'
        actionServer.publish_feedback(actionFeedback)
        process_info.publish(actionFeedback.smach_info)

        # 2. 任务管理器
        if test:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'succeeded', 'no_xml_loaded':'TASK_MANAGER', 'lever':'DETECT_LEVER', 'button2':'RECOGNITION2', 'button_with_state2':'RECOGNITION_WITH_SVM2'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
            
        else:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'RETURN_TO_ORIGIN2', 'no_xml_loaded':'TASK_MANAGER', 'lever':'DETECT_LEVER', 
                               'button1':'RECOGNITION1', 'button2':'RECOGNITION2', 'button_with_state1':'RECOGNITION_WITH_SVM1', 'button_with_state2':'RECOGNITION_WITH_SVM2',
                               'screen':'DETECT_SCREEN','screen_return':'SCREEN_RETURN', 'light':'LIGHT_DETECTION',
                               'manual':'TASK_MANAGER'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
        # smach_ros里的ServiceState只有succeeded、aborted、preempted三种状态


        ######## 3.油门杆检测 ########
        if test:
            @smach.cb_interface(input_keys=['lever_input'], output_keys=['result'], outcomes=['succeeded'])
            def lever_response_cb(userdata, response):
                if userdata.lever_input == '1':
                    userdata.result = True
                else:
                    userdata.result = False
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
                global tic
                tic = time()
                actionFeedback.smach_info = 'lever detect started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                lever_goal = LeverDetectGoal()
                print(userdata.lever_input[0])
                lever_goal.handle_command = str(userdata.lever_input[0])+'-'+str(userdata.lever_input[1])+'-'+str(userdata.lever_input[2])
                return lever_goal
            def lever_result_cb(userdata, status, result): 
                # global toc
                # toc = time()
                # rospy.logwarn("lever time")
                # rospy.logwarn(toc-tic)
                actionFeedback.smach_info = 'lever detect finished'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                sleep(2)
                return 'succeeded'
            StateMachine.add('DETECT_LEVER',
                SimpleActionState('air_handle', LeverDetectAction,
                    goal_cb=lever_goal_cb,
                    result_cb=lever_result_cb),
                transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_operation_info'})



        # 4. 视觉识别（嵌套状态机）
        button_detect_1 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose1','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose1'])
        with button_detect_1:
            ## 4.1 到观测位置
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
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    global tic
                    tic = time()
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
                    SimpleActionState('observe_move_server_1', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'YOLOICP'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose1'})


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
                    transitions={'succeeded':'succeeded'},
                    remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})
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
                    rospy.logwarn("localization table result is: ")
                    rospy.logwarn(response.pose_in_camera)
                    # input('input to continue')
                    userdata.pose_in_camera = response.pose_in_camera
                    userdata.state = [2, int(userdata.icp_input[3])]
                    return
                StateMachine.add('YOLOICP',
                    ServiceState('/hangdian/info/local_tables', LocalTable,
                        request_cb=icp_request_cb,
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose', 'state':'ud_status'})
        StateMachine.add('RECOGNITION1',button_detect_1, {'succeeded':'ARM_DETECT1'})


        button_detect_2 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose2','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose2'])
        with button_detect_2:
            ## 4.1 到观测位置
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
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    global tic
                    tic = time()
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
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})


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
                    transitions={'succeeded':'succeeded'},
                    remapping = {'button_index':'ud_operation_info', 'message':'ud_pose'})
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
                    rospy.logwarn("localization table result is: ")
                    rospy.logwarn(response.pose_in_camera)
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
        StateMachine.add('RECOGNITION2',button_detect_2, {'succeeded':'ARM_DETECT2'})


        button_detect_with_svm1 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose1','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose1'])
        with button_detect_with_svm1:
            ## 4.1 到观测位置
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
                    SimpleActionState('observe_move_server_1', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'YOLOICP'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose1'})


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
                @smach.cb_interface(input_keys=['icp_input'], output_keys=['pose_in_camera'])
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    rospy.logwarn("localization table result is: ")
                    rospy.logwarn(response.pose_in_camera)
                    # input('input to continue')
                    userdata.pose_in_camera = response.pose_in_camera
                    # userdata.state = [2, int(userdata.icp_input[3])]
                    return
                StateMachine.add('YOLOICP',
                    ServiceState('/hangdian/info/local_tables', LocalTable,
                        request_cb=icp_request_cb,
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'SVM'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose'})

            ## 4.3 状态识别
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
                    sleep(2)
                    return svm_request
                @smach.cb_interface(input_keys=['svm_input'], output_keys=['states'])
                def svm_response_cb(userdata, response):
                    actionFeedback.smach_info = 'svm detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    rospy.logwarn("svm result is: ")
                    rospy.logwarn(response.state)
                    userdata.states = [response.state, int(userdata.svm_input[3])]
                    return
                StateMachine.add('SVM',
                    ServiceState('/svm', ButtonDetectState,
                        request_cb=svm_request_cb,
                        response_cb=svm_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'svm_input':'ud_operation_info', 'states':'ud_status'})
        StateMachine.add('RECOGNITION_WITH_SVM1',button_detect_with_svm1, {'succeeded':'ARM_DETECT1'})


        button_detect_with_svm2 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose2','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose2'])
        with button_detect_with_svm2:
            ## 4.1 到观测位置
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
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})


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
                @smach.cb_interface(input_keys=['icp_input'], output_keys=['pose_in_camera'])
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    rospy.logwarn("localization table result is: ")
                    rospy.logwarn(response.pose_in_camera)
                    # input('input to continue')
                    userdata.pose_in_camera = response.pose_in_camera
                    # userdata.state = [2, int(userdata.icp_input[3])]
                    return
                StateMachine.add('YOLOICP',
                    ServiceState('/hangdian/info/local_tables', LocalTable,
                        request_cb=icp_request_cb,
                        response_cb=icp_response_callback),
                    transitions = {'succeeded':'SVM'},
                    remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose'})

            ## 4.3 状态识别
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
                    rospy.logwarn("svm result is: ")
                    rospy.logwarn(response.state)
                    userdata.states = [response.state, userdata.svm_input[3]]
                    return
                StateMachine.add('SVM',
                    ServiceState('/svm', ButtonDetectState,
                        request_cb=svm_request_cb,
                        response_cb=svm_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'svm_input':'ud_operation_info', 'states':'ud_status'})
        StateMachine.add('RECOGNITION_WITH_SVM2',button_detect_with_svm2, {'succeeded':'ARM_DETECT2'})
        
        # 5. 机械臂检测(用action)
        if test:
            def arm_response_cb(userdata, response):
                actionFeedback.smach_info = 'arm detect'
                actionServer.publish_feedback(actionFeedback)
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('ARM_DETECT2', 
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                    response_cb=arm_response_cb), 
                transitions = {'succeeded':'TASK_MANAGER'})# succeeded should be TASK_M in real
            # StateMachine.add('ARM_DETECT', 
            #     SimpleActionState('detect_with_arm_gripper', ButtonManipulateAction, goal = manipulateGoal), 
            #     transitions = {'succeeded':'succeeded'}) 
        else:
            StateMachine.add('ARM_DETECT2', 
                SimpleActionState('manipulate_controller_iiwa2', ButtonManipulateAction, 
                    goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
                transitions = {'succeeded':'TASK_MANAGER'},
                remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
        
        # 每次动完1号机械臂就返回18中间点
        StateMachine.add('ARM_DETECT1', 
            SimpleActionState('manipulate_controller_iiwa1', ButtonManipulateAction, 
                goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
            transitions = {'succeeded':'RETURN_TO_ORIGIN1'},
            remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
        


        # 6. 图像识别
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info', 'ud_current_pose2'], output_keys=['ud_current_pose2'])
        with image_detect:
            ## 6.1 到观测位置（如果需要）
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
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    actionFeedback.smach_info = 'go to observe pose started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    observe_goal = ButtonObserveMoveGoal()
                    print(userdata.observe_input[0])
                    # 5、18表示是图像流，机械臂不用移动
                    if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                        observe_goal.button_index = [userdata.current_pose, userdata.current_pose]
                    # 其余是2号面板，机械臂需要从当前位置移动过去
                    else:
                        observe_goal.button_index = [userdata.current_pose, 2]
                    return observe_goal
                @smach.cb_interface(input_keys=['observe_input', 'current_pose'], output_keys=['current_pose'])
                def observe_result_cb(userdata, status, result): 
                    # TODO:
                    if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                        userdata.current_pose = userdata.current_pose
                    else:
                        userdata.current_pose = 2
                    actionFeedback.smach_info = 'observe pose reached'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    return 'succeeded'
                StateMachine.add('OBSERVE_POSITION',
                    SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'DETECT_IMAGE'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理器
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


        image_detect_return = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info', 'ud_current_pose2'], 
            output_keys=['ud_current_pose2', 'ud_operation_info'])
        with image_detect_return:
            ## 6.1 到观测位置（如果需要）
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
                @smach.cb_interface(input_keys=['observe_input','current_pose'])
                def observe_goal_cb(userdata, goal):  
                    actionFeedback.smach_info = 'go to observe pose started'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    observe_goal = ButtonObserveMoveGoal()
                    print(userdata.observe_input[0])
                    # 5、18表示是图像流，机械臂不用移动
                    if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                        observe_goal.button_index = [userdata.current_pose, userdata.current_pose]
                    # 其余是2号面板，机械臂需要从当前位置移动过去
                    else:
                        observe_goal.button_index = [userdata.current_pose, 2]
                    return observe_goal
                @smach.cb_interface(input_keys=['observe_input', 'current_pose'], output_keys=['current_pose'])
                def observe_result_cb(userdata, status, result): 
                    # TODO:
                    if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                        userdata.current_pose = userdata.current_pose
                    else:
                        userdata.current_pose = 2
                    actionFeedback.smach_info = 'observe pose reached'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    return 'succeeded'
                StateMachine.add('OBSERVE_POSITION',
                    SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                        goal_cb=observe_goal_cb,
                        result_cb=observe_result_cb),
                    transitions = {'succeeded':'DETECT_IMAGE_RETURN'},# succeeded should be TASK_M in real
                    remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理器
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
                    image_request = ScreenDetectReturnRequest()
                    image_request.param_b = int(userdata.image_input[0])#b
                    image_request.param_c = int(userdata.image_input[1])#c
                    image_request.param_d = userdata.image_input[2]#d
                    image_request.param_e = int(userdata.image_input[3])#e
                    image_request.param_f = int(userdata.image_input[4])#f
                    image_request.param_g = int(userdata.image_input[5])#g
                    # print(image_request.param_b+image_request.param_c+int(image_request.param_d)+image_request.param_e+image_request.param_f+image_request.param_g)
                    return image_request
                @smach.cb_interface(output_keys=['image_return'])
                def image_response_cb(userdata, response):
                    actionFeedback.smach_info = 'image detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    rospy.logwarn("time detection result is: ")
                    rospy.logwarn(response.time[0])
                    rospy.logwarn(response.time[1])
                    a1 = int(response.time[0]) / 10 
                    a2 = int(response.time[0]) % 10
                    b1 = (int(response.time[1])+3) / 10
                    b2 = (int(response.time[1])+3) % 10
                    userdata.image_return = [2, 4, 6, number[a1], number[a2], number[b1], number[b2], number[0], number[0]] 
                    return
                StateMachine.add('DETECT_IMAGE_RETURN',
                    ServiceState('/halcon_detection_return_service', ScreenDetectReturn,
                        request_cb=image_request_cb,
                        response_cb=image_response_cb),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'image_input':'ud_operation_info','image_return':'ud_operation_info'})
        StateMachine.add('SCREEN_RETURN', image_detect_return, {'succeeded':'RECOGNITION2'})


        # 7. 按钮灯光检测
        @smach.cb_interface(input_keys=['light_input'])
        def svm_request_cb(userdata, request):
            actionFeedback.smach_info = 'light detect started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            svm_request = ButtonDetectStateRequest()
            svm_request.panel_index = int(userdata.light_input[0])
            svm_request.button_index = userdata.light_input[2]
            return svm_request
        @smach.cb_interface(input_keys=['light_input'])
        def svm_response_cb(userdata, response):
            actionFeedback.smach_info = 'light detect finished'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            if response.state ==  int(userdata.light_input[3]):
                rospy.logwarn("light result is: ")
                rospy.logwarn(userdata.light_input[3])
            else:
                rospy.logerr("light result is not: ")
                rospy.logerr(userdata.light_input[3])
            # userdata.states = [response.state, int(userdata.svm_input[3])]
            return
        StateMachine.add('LIGHT_DETECTION',
            ServiceState('/svm', ButtonDetectState,
                request_cb=svm_request_cb,
                response_cb=svm_response_cb),
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'light_input':'ud_operation_info'})


        # 8. 1号右臂归位
        @smach.cb_interface(input_keys=['current_pose'])
        def observe_goal_cb(userdata, goal):  
            actionFeedback.smach_info = 'go to origin pose 18 started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            observe_goal = ButtonObserveMoveGoal()
            observe_goal.button_index = [userdata.current_pose, 18]
            return observe_goal
        @smach.cb_interface(output_keys=['current_pose'])
        def observe_result_cb(userdata, status, result): 
            # sleep(1.0)
            userdata.current_pose = 18
            actionFeedback.smach_info = 'origin pose reached'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            return 'succeeded'
        StateMachine.add('RETURN_TO_ORIGIN1',
            SimpleActionState('observe_move_server_1', ButtonObserveMoveAction,
                goal_cb=observe_goal_cb,
                result_cb=observe_result_cb),
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'current_pose':'ud_current_pose1'})


        # 9. 2号左臂归位
        @smach.cb_interface(input_keys=['current_pose'])
        def observe_goal_cb(userdata, goal):  
            # actionFeedback.smach_info = 'go to origin pose 11 started'
            actionFeedback.smach_info = 'go to origin pose 4 started'

            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            observe_goal = ButtonObserveMoveGoal()
            observe_goal.button_index = [userdata.current_pose, 11]
            return observe_goal
        @smach.cb_interface(output_keys=['current_pose'])
        def observe_result_cb(userdata, status, result): 
            # sleep(1.0)
            userdata.current_pose =11
            actionFeedback.smach_info = 'origin pose reached'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            return 'succeeded'
        StateMachine.add('RETURN_TO_ORIGIN2',
            SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                goal_cb=observe_goal_cb,
                result_cb=observe_result_cb),
            transitions = {'succeeded':'succeeded'},
            remapping = {'current_pose':'ud_current_pose2'})

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
    rospy.init_node('hangdian_state_machine_wuhu')
    global actionServer
    actionServer = actionlib.SimpleActionServer('central_control', CentralControlRunAction, execute, auto_start=False)
    actionServer.start()
    rospy.loginfo('central control server started')

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()
