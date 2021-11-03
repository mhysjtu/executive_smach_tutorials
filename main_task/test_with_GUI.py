#!/usr/bin/env python2
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

from re import U
from time import sleep, time
from actionlib.action_client import TerminalState
import rospy
import actionlib
import threading
import random

import smach
from smach import StateMachine, State
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
from geometry_msgs.msg import Pose
# 需要source才能找到�?定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse
from hangdian_msgs.srv import ScreenDetect, ScreenDetectRequest, TaskManage, TaskManageResponse
from hangdian_msgs.msg import LeverDetectAction, LeverDetectGoal, ButtonManipulateAction, ButtonManipulateGoal
from hangdian_msgs.msg import CentralControlRunAction, CentralControlRunResult, CentralControlRunFeedback

#std_srvs.srv.SetBoolRequest
# from iiwa_msgs.msg import ControlMode
# from iiwa_msgs.srv import ConfigureControlMode
# import iiwa_msgs.msg

central_control_action_result = False

class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'lever', 'button', 'screen'],
                             output_keys=['operation_info', 'operate_type', 'panel', 'objects'])
        self.outcome = 'succeeded'
    def execute(self, userdata):
        actionFeedback.smach_info = 'task manager'
        actionServer.publish_feedback(actionFeedback)
        self.outcome= 'succeeded'
        rospy.wait_for_service('/hangdian/task_manager_service')
        try:
            getTask = rospy.ServiceProxy('/hangdian/task_manager_service', TaskManage)
            taskResponse = getTask()
            have_task = taskResponse.have_task
            if not have_task:
                print("All task have been completed !!!")
                self.outcome = 'succeeded'
                global central_control_action_result
                central_control_action_result = True
                actionFeedback.smach_info = 'task finished'
                actionServer.publish_feedback(actionFeedback)
                return self.outcome
            else:
                #最后会是一个函数，输入service返回的response——字符串，得到操作种类、面板序号、开关种类、开关序号：
                print(taskResponse.module)
                # userdata.operation_info = [taskResponse.panel, taskResponse.operate_type, taskResponse.objects]
                # # userdata.module = taskResponse.module
                userdata.operate_type = taskResponse.operate_type
                userdata.panel = taskResponse.panel
                userdata.objects = taskResponse.objects
                
                if taskResponse.module == 1:
                    print("To detect button !!!")
                    self.outcome = 'button'
                    userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                        int(taskResponse.objects[0])]
                    #if userdata.operate_type == 4:
                    #   userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type)]
                    #   for i in taskResponse.objects
                    #       userdata.operation_info.push(int(i))
                elif taskResponse.module == 2:
                    print("To detect lever !!!")
                    self.outcome = 'lever'
                    userdata.operation_info = [taskResponse.panel, int(taskResponse.operate_type), 
                        int(taskResponse.objects[0])]
                    # print(userdata.operation_info)
                elif taskResponse.module == 3:
                    print("To detect screen !!!")
                    self.outcome = 'screen'
                    userdata.operation_info = [taskResponse.panel, taskResponse.operate_type, taskResponse.objects]
                
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
        # 0. �?�?�?
        global actionFeedback
        actionFeedback = CentralControlRunFeedback()
        global actionResult
        actionResult = CentralControlRunResult()
        actionResult.result = True

        # 1. �?检
        # check if sensors are OK
        StateMachine.add('CHECK',
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger),
                {'succeeded':'TASK_MANAGER', 'aborted':'aborted'})
        actionFeedback.smach_info = 'self check'
        actionServer.publish_feedback(actionFeedback)


        # 2. 任务管理�?
        ## 如果任务�?完成，继�?检�?
        ## 任务完成，则到总sm的succeed状�?
        # check if sensors are OK
        if test:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'succeeded', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
            
        else:
            StateMachine.add('TASK_MANAGER',
                TaskServiceCB(),
                transitions = {'succeeded':'succeeded', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
                remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                             'panel':'ud_panel', 'objects':'ud_objects'})
        # smach_ros里的ServiceState�?有succeeded、aborted、preempted三�?�状�?

        # 3. 操纵杆�?��?
        # check if sensors are OK
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
                lever_goal = LeverDetectGoal()
                print(userdata.lever_input[0])
                if userdata.lever_input[1] == 1:
                    lever_goal.handle_command = 'JSA'
                return lever_goal
            def lever_result_cb(userdata, status, result): 
                actionFeedback.smach_info = 'lever detect finished'
                actionServer.publish_feedback(actionFeedback)
                return 'succeeded'
            StateMachine.add('DETECT_LEVER',
                SimpleActionState('air_handle', LeverDetectAction,
                    goal_cb=lever_goal_cb,
                    result_cb=lever_result_cb),
                transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
                remapping = {'lever_input':'ud_operation_info'})
            
            # @smach.cb_interface(input_keys=['lever_input'], output_keys=['result'], outcomes=['succeeded'])
            # def lever_response_cb(userdata, response):
            #     if userdata.lever_input == '1':
            #         userdata.result = True
            #     else:
            #         userdata.result = False
            #     # status = userdata.result # 除了不能print output_keys, �?的也没问题了
            #     # print userdata.final_result
            #     actionFeedback.smach_info = 'lever detect'
            #     actionServer.publish_feedback(actionFeedback)
            #     return
            # StateMachine.add('DETECT_LEVER',
            #     ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
            #         response_cb=lever_response_cb),
            #     transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
            #     remapping = {'lever_input':'ud_operate_type'} # userdata in viewer are only ud_xxx and final_xxx
            #     )


        # 4. 视�?�识�?（嵌套状态机�?
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info'], output_keys=['ud_pose', 'ud_status'])
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
                    icp_request = ButtonDetectPoseRequest()
                    icp_request.button_index = int(userdata.icp_input[0])
                    print(icp_request.button_index)
                    return icp_request
                def icp_response_callback(userdata, response):
                    actionFeedback.smach_info = 'icp detect finished'
                    actionServer.publish_feedback(actionFeedback)
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
                    svm_request = ButtonDetectStateRequest()
                    svm_request.panel_index = int(userdata.svm_input[0])
                    svm_request.button_index = userdata.svm_input[2]
                    return svm_request
                def svm_response_callback(userdata, response):
                    actionFeedback.smach_info = 'svm detect finished'
                    actionServer.publish_feedback(actionFeedback)
                    print("svm result is: ")
                    print(response.state)
                    return
                StateMachine.add('SVM',
                    ServiceState('/svm', ButtonDetectState,
                        request_cb=svm_request_cb,
                        response_slots=['state'],
                        response_cb=svm_response_callback),
                    transitions = {'succeeded':'succeeded'},
                    remapping = {'svm_input':'ud_operation_info', 'state':'ud_status'})
        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})
        
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
                #     transitions = {'succeeded':'DETECT_IMAGE'},
                #     remapping = {'button_index':'ud_index_of_button'})
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
                    image_request = ScreenDetectRequest()
                    image_request.category = int(userdata.image_input[0])
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
