#!/usr/bin/env python2
#-*- coding: UTF-8 -*-
"""
Description:
    从check_single_button.py复制
    - 删去了test(若运行出现问题, 则重新运行check_single_button.py)
    - 补充注释
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

# 全局变量定义
# =================================================================
# 用于各任务的计时
tic = time()
toc = time()
# 判断action是否执行成功; 只有全部任务执行完毕, action才会返回true
central_control_action_result = False
# 2号面板0-9数字键与按键编号的映射
number = {0:27, 1:8, 2:9, 3:10, 4:14, 5:15, 6:16, 7:20, 8:21, 9:22 }
# 执行过程消息的publisher, 发布给人机界面
process_info = rospy.Publisher('/hangdian/centralControlInfo', String, queue_size=10)

# "任务管理"状态
# =================================================================
class TaskServiceCB(smach.State):
    def __init__(self):
        # 定义本状态的所有 结果 和 输出值
        smach.State.__init__(self, outcomes=['succeeded', 'no_xml_loaded', 'lever', 'aborted', 'button1', 'button2', 'button_with_state1', 'button_with_state2', 'screen', 'screen_return', 'light', 'manual'],
                             output_keys=['operation_info', 'operate_type', 'panel', 'objects'])
        # 状态的默认结果
        self.outcome = 'succeeded'
    def execute(self, userdata):
        self.outcome= 'succeeded'
        # 等待解析节点的服务可用
        rospy.wait_for_service('/hangdian/task_manager_service')
        try:
            # 发送service请求
            getTask = rospy.ServiceProxy('/hangdian/task_manager_service', TaskManage)
            taskResponse = getTask(" ") #人机界面已经请求过xml文件，之后就都发空的service requset

            # 处理service的响应结果
            loaded_xml = taskResponse.load_xml
            # xml文件载入成功
            if loaded_xml:
                have_task = taskResponse.have_task
                # 若没有剩余任务，即任务全部执行完毕
                if not have_task:
                    global toc
                    global tic
                    toc = time()
                    rospy.logwarn("task time")
                    rospy.logwarn(toc-tic)

                    print("All task have been completed !!!")
                    global central_control_action_result
                    central_control_action_result = True
                    actionFeedback.smach_info = 'task finished;'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    self.outcome = 'succeeded'
                    return self.outcome
                # 仍有剩余任务
                else:
                    # TODO：改为一个函数---输入service返回的response——字符串，得到操作种类、面板序号、开关种类、开关序号：
                    userdata.operate_type = taskResponse.operate_type
                    userdata.panel = taskResponse.panel
                    userdata.objects = taskResponse.objects

                    actionFeedback.smach_info = 'task manager;'+str(taskResponse.module)+';'+str(taskResponse.operate_type)+';'+str(taskResponse.panel)+';'+str(taskResponse.objects)+';'
                    actionServer.publish_feedback(actionFeedback)
                    process_info.publish(actionFeedback.smach_info)
                    
                    # 计时
                    global toc
                    global tic
                    toc = time()
                    rospy.logwarn("task time")
                    rospy.logwarn(toc-tic)

                    # 根据 taskResponse.module 的不同，得到不同的状态结果
                    # 右臂检测
                    if taskResponse.module == 0:
                        print("To detect right button !!!")
                        self.outcome = 'button1'
                        
                        # 根据操作类型，执行不同状态
                        # 拨杆开关
                        if taskResponse.operate_type == 2:
                            # 正则数组：[面板编号，操作类型，开关编号，开关目标状态]
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                            self.outcome = 'button1' # 为了测试，此处人为输入开关状态；若需调用svm识别状态，则要改为'button_with_state1'
                        # 连续按压按钮开关
                        elif taskResponse.operate_type == 4:
                            # 正则数组：[面板编号，操作类型，连续按数量，开关1, ...]
                            a = [int(taskResponse.panel), int(taskResponse.operate_type)]
                            b = [int(x) for x in taskResponse.objects]
                            userdata.operation_info = a+b 
                            self.outcome = 'button1'
                        # 其余类型（按压单个按钮）
                        else:
                            # 正则数组：[面板编号，操作类型，开关，0] TODO:0可能是多余的
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]),0]
                            self.outcome = 'button1'
                    # 左臂检测
                    elif taskResponse.module == 1:
                        print("To detect left button !!!")
                        self.outcome = 'button2'

                        # 根据操作类型，执行不同状态
                        # 需要检测状态的开关
                        if taskResponse.operate_type == 2 or taskResponse.operate_type == 3 or taskResponse.operate_type == 6 or taskResponse.operate_type == 5: 
                            # 正则数组：[面板编号，操作类型，开关编号，开关目标状态]
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]
                            self.outcome = 'button2' # 为了测试，此处人为输入开关状态；若需调用svm识别状态，则要改为'button_with_state2'
                        # 连续按压按钮开关
                        elif taskResponse.operate_type == 4: 
                            # 正则数组：[面板编号，操作类型，连续按数量，开关1, ...]
                            a = [int(taskResponse.panel), int(taskResponse.operate_type)]
                            b = [int(x) for x in taskResponse.objects]
                            userdata.operation_info = a+b 
                            self.outcome = 'button2'
                        # 其余类型（按压单个按钮）
                        else:
                            # 正则数组：[面板编号，操作类型，开关，0] TODO:0可能是多余的
                            userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]),0]
                            self.outcome = 'button2'
                    # 操纵杆检测
                    elif taskResponse.module == 2:
                        print("To detect lever !!!")
                        self.outcome = 'lever'
                        # 正则数组：[驾驶杆or油门杆，哪个开关，开关状态]
                        userdata.operation_info = [int(taskResponse.operate_type),taskResponse.panel,
                            int(taskResponse.objects[0])]
                    # 图像检测 
                    elif taskResponse.module == 3:
                        # 等待图像稳定
                        sleep(0.5)
                        print("To detect image !!!")
                        # 正则数组：[b,c,d,e,f,g...]
                        a = [int(taskResponse.operate_type), int(taskResponse.panel)]
                        b = [x for x in taskResponse.objects]
                        userdata.operation_info = a + b

                        # mhy for mid term exam, 现场使用需换为 screen
                        # self.outcome = 'screen'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
                    # 灯光检测    
                    elif taskResponse.module == 4:
                        print("To detect light !!!")
                        # 正则数组：[面板编号，操作类型(仅为1)，开关编号，开关期望状态]
                        userdata.operation_info = [int(taskResponse.panel), int(taskResponse.operate_type), 
                                int(taskResponse.objects[0]), int(taskResponse.objects[1])]

                        # mhy for mid term exam, 现场使用需换为 light
                        # self.outcome = 'light'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
                    # 图像检测返回结果
                    elif taskResponse.module == 5:
                        # 等待图像稳定
                        sleep(0.5)
                        print("To detect image and return result!!!")
                        # 正则数组：[b,c,d,e,f,g...]
                        a = [int(taskResponse.operate_type), int(taskResponse.panel)]
                        b = [x for x in taskResponse.objects]
                        userdata.operation_info = a + b
                        
                        # mhy for mid term exam, 现场使用需换为 screen_return
                        # self.outcome = 'screen_return'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
                    # 手动操作
                    elif taskResponse.module == 6:
                        rospy.logwarn("请手动操作：")
                        rospy.logwarn(taskResponse.panel)
                        rospy.logwarn('操作完毕后，请输入3,并按回车继续。。。')
                        input()
                        
                        # mhy for mid term exam, 现场使用需换为 manual
                        # self.outcome = 'manual'
                        self.outcome = 'no_xml_loaded'
                        # mhy for mid term exam
            # 若xml文件未载入
            else:
                self.outcome = 'no_xml_loaded'
            return self.outcome
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.outcome = 'aborted'
            return self.outcome

# action的execute回调函数
# =================================================================
def execute(goal):
    global central_control_action_result

    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm0:
        # 0. action的feedback和result全局变量定义
        global actionFeedback
        actionFeedback = CentralControlRunFeedback()
        global actionResult
        actionResult = CentralControlRunResult()
        actionResult.result = True

        # 1. 'CHECK'状态  ==>  'TASK_MANAGER'
        # @描述：设置机械臂初始面板位置。若机械臂初始位置改变，则需要更改18，11的值
        # @输出：            局部userdata   --> 全局userdata
        #       右臂初始位置: current_pose1 --> ud_current_pose1
        #       左臂初始位置: current_pose2 --> ud_current_pose2
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
        actionFeedback.smach_info = 'set initial panel'
        actionServer.publish_feedback(actionFeedback)
        process_info.publish(actionFeedback.smach_info)


        # 2. 'TASK_MANAGER'状态
        # @描述：任务管理器，管理各任务的切换，切换方法见TaskServiceCB类的execute成员函数
        #       各状态映射见下方的transitions字典，key代表TASK_MANAGER状态的结果，value代表下一个执行的状态
        # @输出：                      局部userdata    --> 全局userdata
        #       各自模块接受的正则数组:   operation_info --> ud_operation_info
        #       开关操作类型:           operate_type   --> ud_operate_type
        #       开关所在面板:           panel          --> ud_panel
        #       操作的开关（一个或多个）: objects        --> ud_objects
        StateMachine.add('TASK_MANAGER',
            TaskServiceCB(),
            transitions = {'succeeded':'RETURN_TO_ORIGIN2', 'no_xml_loaded':'TASK_MANAGER', 'lever':'DETECT_LEVER', 
                            'button1':'RECOGNITION1', 'button2':'RECOGNITION2', 'button_with_state1':'RECOGNITION_WITH_SVM1', 'button_with_state2':'RECOGNITION_WITH_SVM2',
                            'screen':'DETECT_SCREEN','screen_return':'SCREEN_RETURN', 'light':'LIGHT_DETECTION',
                            'manual':'TASK_MANAGER'},
            remapping = {'operation_info':'ud_operation_info', 'operate_type':'ud_operate_type', 
                            'panel':'ud_panel', 'objects':'ud_objects'})


        # 3. 'DETECT_LEVER'状态  ==>  'TASK_MANAGER'
        # @描述：调用操作杆检测模块
        # @输入:                        全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:   ud_operation_info --> lever_input
        @smach.cb_interface(input_keys=['lever_input'])
        def lever_goal_cb(userdata, goal):
            global tic
            tic = time()

            actionFeedback.smach_info = 'lever detect started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)

            lever_goal = LeverDetectGoal()
            print(userdata.lever_input[0])
            # 调用参数：[x-x-x]
            lever_goal.handle_command = str(userdata.lever_input[0])+'-'+str(userdata.lever_input[1])+'-'+str(userdata.lever_input[2])
            return lever_goal
        def lever_result_cb(userdata, status, result): 
            actionFeedback.smach_info = 'lever detect finished'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            # 防止连续调用时执行太快
            sleep(2)
            return 'succeeded'
        StateMachine.add('DETECT_LEVER',
            SimpleActionState('air_handle', LeverDetectAction,
                goal_cb=lever_goal_cb,
                result_cb=lever_result_cb),
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'lever_input':'ud_operation_info'})


        # 4A. 'RECOGNITION1'嵌套状态  ==>  'ARM_DETECT1'
        # @描述：1号机械臂（右臂）运动至观测位置、再获得待操作面板相对于相机的pose
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / icp_input
        #       手臂之前所在观测位置:          ud_current_pose1  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
        #       开关当前状态：                state             --> ud_status
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose1
        button_detect_1 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose1','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose1'])
        with button_detect_1:
            # 4A.1 'OBSERVE_POSITION'子状态  ==>  'YOLOICP'
            # @描述：1号机械臂到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:  ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose1  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose1
            @smach.cb_interface(input_keys=['observe_input','current_pose'])
            def observe_goal_cb(userdata, goal):
                global tic
                tic = time()

                actionFeedback.smach_info = 'go to observe pose started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                observe_goal = ButtonObserveMoveGoal()
                print(userdata.observe_input[0])
                # 调用参数：[当前位置, 目标位置]
                observe_goal.button_index = [userdata.current_pose, userdata.observe_input[0]]
                return observe_goal
            @smach.cb_interface(input_keys=['observe_input'], output_keys=['current_pose'])
            def observe_result_cb(userdata, status, result): 
                # 运动完成，更改当前位置
                userdata.current_pose = userdata.observe_input[0]

                actionFeedback.smach_info = 'observe pose reached'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                return 'succeeded'
            StateMachine.add('OBSERVE_POSITION',
                SimpleActionState('observe_move_server_1', ButtonObserveMoveAction,
                    goal_cb=observe_goal_cb,
                    result_cb=observe_result_cb),
                transitions = {'succeeded':'YOLOICP'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose1'})


            # 4A.2 'YOLOICP'子状态
            # @描述：从本地table中，获取待操作面板相对于1号手臂末端的pose以及人工指定的开关状态
            # @输入：                           全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:      ud_operation_info --> icp_input
            # @输出:                            局部userdata       --> 全局userdata
            #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
            #       开关当前状态：                state             --> ud_status
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

                userdata.pose_in_camera = response.pose_in_camera
                # 第一位是初始状态，需要根据实际开关状态更改；第二位是期望的开关状态
                userdata.state = [2, int(userdata.icp_input[3])]
                return
            StateMachine.add('YOLOICP',
                ServiceState('/hangdian/info/local_tables', LocalTable,
                    request_cb=icp_request_cb,
                    response_cb=icp_response_callback),
                transitions = {'succeeded':'succeeded'},
                remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose', 'state':'ud_status'})
        StateMachine.add('RECOGNITION1',button_detect_1, {'succeeded':'ARM_DETECT1'})


        # 5A. 'RECOGNITION2'嵌套状态  ==>  'ARM_DETECT2'
        # @描述：2号机械臂（左臂）运动至观测位置、再获得待操作面板相对于相机的pose
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / icp_input
        #       手臂之前所在观测位置:          ud_current_pose2  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
        #       开关当前状态：                state             --> ud_status
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose2
        button_detect_2 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose2','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose2'])
        with button_detect_2:
            # 5A.1 'OBSERVE_POSITION'子状态  ==>  'YOLOICP'
            # @描述：2号机械臂到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:  ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose2  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose2
            @smach.cb_interface(input_keys=['observe_input','current_pose'])
            def observe_goal_cb(userdata, goal):  
                global tic
                tic = time()

                actionFeedback.smach_info = 'go to observe pose started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                observe_goal = ButtonObserveMoveGoal()
                print(userdata.observe_input[0])
                # 调用参数：[当前位置, 目标位置]
                observe_goal.button_index = [userdata.current_pose, userdata.observe_input[0]]
                return observe_goal
            @smach.cb_interface(input_keys=['observe_input'], output_keys=['current_pose'])
            def observe_result_cb(userdata, status, result): 
                # 运动完成，更改当前位置
                userdata.current_pose = userdata.observe_input[0]

                actionFeedback.smach_info = 'observe pose reached'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                return 'succeeded'
            StateMachine.add('OBSERVE_POSITION',
                SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                    goal_cb=observe_goal_cb,
                    result_cb=observe_result_cb),
                transitions = {'succeeded':'YOLOICP'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})


            # 5A.2 'YOLOICP'子状态
            # @描述：从本地table中，获取待操作面板相对于2号手臂末端的pose以及人工指定的开关状态
            # @输入：                           全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:      ud_operation_info --> icp_input
            # @输出:                            局部userdata       --> 全局userdata
            #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
            #       开关当前状态：                state             --> ud_status
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
                
                # 第一位是初始状态，需要根据实际开关状态更改；第二位是期望的开关状态
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


        # 4B. 'RECOGNITION_WITH_SVM1'嵌套状态  ==>  'ARM_DETECT1'
        # @描述：1号机械臂（右臂）运动至观测位置、再获得待操作面板相对于相机的pose、最后通过SVM识别开关状态
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / icp_input / svm_input
        #       手臂之前所在观测位置:          ud_current_pose1  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
        #       开关当前状态：                state             --> ud_status
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose1
        button_detect_with_svm1 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose1','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose1'])
        with button_detect_with_svm1:
            # 4B.1 'OBSERVE_POSITION'子状态  ==>  'YOLOICP'
            # @描述：1号机械臂到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组: ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose1  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose1
            @smach.cb_interface(input_keys=['observe_input','current_pose'])
            def observe_goal_cb(userdata, goal):  
                actionFeedback.smach_info = 'go to observe pose started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                observe_goal = ButtonObserveMoveGoal()
                print(userdata.observe_input[0])
                # 调用参数：[当前位置, 目标位置]
                observe_goal.button_index = [userdata.current_pose, userdata.observe_input[0]]
                return observe_goal
            @smach.cb_interface(input_keys=['observe_input'], output_keys=['current_pose'])
            def observe_result_cb(userdata, status, result): 
                # 运动完成，更改当前位置
                userdata.current_pose = userdata.observe_input[0]

                actionFeedback.smach_info = 'observe pose reached'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                return 'succeeded'
            StateMachine.add('OBSERVE_POSITION',
                SimpleActionState('observe_move_server_1', ButtonObserveMoveAction,
                    goal_cb=observe_goal_cb,
                    result_cb=observe_result_cb),
                transitions = {'succeeded':'YOLOICP'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose1'})


            # 4B.2 'YOLOICP'子状态  ==>  'SVM'
            # @描述：从本地table中，获取待操作面板相对于手臂末端的pose以及人工指定的开关状态
            # @输入：                          全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:     ud_operation_info --> icp_input
            # @输出:                           局部userdata       --> 全局userdata
            #       待操作面板相对于相机的pose:   pose_in_camera    --> ud_pose
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

                userdata.pose_in_camera = response.pose_in_camera
                return
            StateMachine.add('YOLOICP',
                ServiceState('/hangdian/info/local_tables', LocalTable,
                    request_cb=icp_request_cb,
                    response_cb=icp_response_callback),
                transitions = {'succeeded':'SVM'},
                remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose'})


            # 4B.3 'SVM'子状态
            # @描述：调用开关状态检测模块，识别开关状态
            # @输入：                           全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:      ud_operation_info --> svm_input
            # @输出:                            局部userdata       --> 全局userdata
            #       开关当前状态：                states             --> ud_status
            @smach.cb_interface(input_keys=['svm_input'])
            def svm_request_cb(userdata, request):
                actionFeedback.smach_info = 'svm detect started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                
                svm_request = ButtonDetectStateRequest()
                svm_request.panel_index = int(userdata.svm_input[0])
                svm_request.button_index = userdata.svm_input[2]
                # 等待相机稳定 TDDO:是否有必要
                sleep(2)
                return svm_request
            @smach.cb_interface(input_keys=['svm_input'], output_keys=['states'])
            def svm_response_cb(userdata, response):
                actionFeedback.smach_info = 'svm detect finished'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                rospy.logwarn("svm result is: ")
                rospy.logwarn(response.state)

                # 调用参数：[当前开关状态（模块调用结果）, 目标开关状态]
                userdata.states = [response.state, int(userdata.svm_input[3])]
                return
            StateMachine.add('SVM',
                ServiceState('/svm', ButtonDetectState,
                    request_cb=svm_request_cb,
                    response_cb=svm_response_cb),
                transitions = {'succeeded':'succeeded'},
                remapping = {'svm_input':'ud_operation_info', 'states':'ud_status'})
        StateMachine.add('RECOGNITION_WITH_SVM1',button_detect_with_svm1, {'succeeded':'ARM_DETECT1'})


        # 5B. 'RECOGNITION_WITH_SVM2'嵌套状态  ==>  'ARM_DETECT2'
        # @描述：2号机械臂（左臂）运动至观测位置、再获得待操作面板相对于相机的pose、最后通过SVM识别开关状态
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / icp_input / svm_input
        #       手臂之前所在观测位置:          ud_current_pose2  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       待操作面板相对于相机的pose:    pose_in_camera    --> ud_pose
        #       开关当前状态：                state             --> ud_status
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose2
        button_detect_with_svm2 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_current_pose2','ud_operation_info'], output_keys=['ud_pose', 'ud_status','ud_current_pose2'])
        with button_detect_with_svm2:
            # 5B.1 'OBSERVE_POSITION'子状态  ==>  'YOLOICP'
            # @描述：1号机械臂到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组: ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose2  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose2
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
                userdata.current_pose = userdata.observe_input[0]

                actionFeedback.smach_info = 'observe pose reached'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                return 'succeeded'
            StateMachine.add('OBSERVE_POSITION',
                SimpleActionState('observe_move_server_2', ButtonObserveMoveAction,
                    goal_cb=observe_goal_cb,
                    result_cb=observe_result_cb),
                transitions = {'succeeded':'YOLOICP'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})


            # 5B.2 'YOLOICP'子状态  ==>  'SVM'
            # @描述：从本地table中，获取待操作面板相对于手臂末端的pose以及人工指定的开关状态
            # @输入:                           全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:     ud_operation_info --> icp_input
            # @输出:                           局部userdata       --> 全局userdata
            #       待操作面板相对于相机的pose:   pose_in_camera    --> ud_pose
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
                
                userdata.pose_in_camera = response.pose_in_camera
                return
            StateMachine.add('YOLOICP',
                ServiceState('/hangdian/info/local_tables', LocalTable,
                    request_cb=icp_request_cb,
                    response_cb=icp_response_callback),
                transitions = {'succeeded':'SVM'},
                remapping = {'icp_input':'ud_operation_info', 'pose_in_camera':'ud_pose'})


            # 5B.3 状态识别
            # @描述：调用开关状态检测模块，识别开关状态
            # @输入：                           全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:     ud_operation_info --> svm_input
            # @输出:                            局部userdata       --> 全局userdata
            #       开关当前状态：                states             --> ud_status
            @smach.cb_interface(input_keys=['svm_input'])
            def svm_request_cb(userdata, request):
                actionFeedback.smach_info = 'svm detect started'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                svm_request = ButtonDetectStateRequest()
                svm_request.panel_index = int(userdata.svm_input[0])
                svm_request.button_index = userdata.svm_input[2]
                # 等待相机稳定 TDDO:是否有必要
                sleep(2)
                return svm_request
            @smach.cb_interface(input_keys=['svm_input'], output_keys=['states'])
            def svm_response_cb(userdata, response):
                actionFeedback.smach_info = 'svm detect finished'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)
                rospy.logwarn("svm result is: ")
                rospy.logwarn(response.state)

                # 调用参数：[当前开关状态（模块调用结果）, 目标开关状态]
                userdata.states = [response.state, userdata.svm_input[3]]
                return
            StateMachine.add('SVM',
                ServiceState('/svm', ButtonDetectState,
                    request_cb=svm_request_cb,
                    response_cb=svm_response_cb),
                transitions = {'succeeded':'succeeded'},
                remapping = {'svm_input':'ud_operation_info', 'states':'ud_status'})
        StateMachine.add('RECOGNITION_WITH_SVM2',button_detect_with_svm2, {'succeeded':'ARM_DETECT2'})
        

        # 6. 'ARM_DETECT2'状态  ==>  'TASK_MANAGER'
        # @描述：调用机械臂检测模块，2号机械臂执行检测任务
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> button_index
        #       待操作面板相对于相机的pose:    ud_pose           --> pose_in_camera
        #       开关当前状态：                ud_status         --> button_state
        StateMachine.add('ARM_DETECT2', 
            SimpleActionState('manipulate_controller_iiwa2', ButtonManipulateAction, 
                goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
        

        # 7. 'ARM_DETECT1'状态  ==>  'RETURN_TO_ORIGIN1'（1号机械臂检测完即执行'回原位'状态，防止与2号碰撞）
        # @描述：调用机械臂检测模块，1号机械臂执行检测任务
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> button_index
        #       待操作面板相对于相机的pose: ud_pose           --> pose_in_camera
        #       开关当前状态：                ud_status         --> button_state
        StateMachine.add('ARM_DETECT1', 
            SimpleActionState('manipulate_controller_iiwa1', ButtonManipulateAction, 
                goal_slots = ['button_index', 'pose_in_camera', 'button_state']), 
            transitions = {'succeeded':'RETURN_TO_ORIGIN1'},
            remapping = {'button_index':'ud_operation_info', 'pose_in_camera':'ud_pose', 'button_state':'ud_status'})
        

        # 8. 'DETECT_SCREEN'嵌套状态  ==>  'TASK_MANAGER'
        # @描述：机械臂移动至观测位置（如有必要），调用显示屏检测模块，识别显示屏图像
        #       2号面板的数码管检测需要2号机械臂移动
        # @输入:                            全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / image_input
        #       手臂之前所在观测位置:          ud_current_pose2  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose2
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info', 'ud_current_pose2'], output_keys=['ud_current_pose2'])
        with image_detect:
            # 8.1 'OBSERVE_POSITION'子状态  ==>  'DETECT_IMAGE'
            # @描述：若需要检测2号面板数码管，则2号机械臂需要运动到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组: ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose2  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose2
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
                # 否则是2号面板，机械臂需要从当前位置移动过去
                else:
                    observe_goal.button_index = [userdata.current_pose, 2]
                return observe_goal
            @smach.cb_interface(input_keys=['observe_input', 'current_pose'], output_keys=['current_pose'])
            def observe_result_cb(userdata, status, result): 
                # 5、18表示是图像流，机械臂仍在原位
                if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                    userdata.current_pose = userdata.current_pose
                # 否则是2号面板，认为机械臂已到位
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
                transitions = {'succeeded':'DETECT_IMAGE'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})

            # 8.2 'DETECT_IMAGE'子状态
            # @描述：调用显示屏检测模块，识别显示屏图像
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组: ud_operation_info --> image_input
            @smach.cb_interface(input_keys=['image_input'])
            def image_request_cb(userdata, request):
                actionFeedback.smach_info = 'image detect'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                image_request = ScreenDetectRequest()
                image_request.param_b = int(userdata.image_input[0])#b
                image_request.param_c = int(userdata.image_input[1])#c
                image_request.param_d = userdata.image_input[2]     #d
                image_request.param_e = int(userdata.image_input[3])#e
                image_request.param_f = int(userdata.image_input[4])#f
                image_request.param_g = int(userdata.image_input[5])#g
                # 检查发送的消息是否正确
                # print(image_request.param_b+image_request.param_c+int(image_request.param_d)+image_request.param_e+image_request.param_f+image_request.param_g)
                return image_request
            def image_response_cb(userdata, response):
                actionFeedback.smach_info = 'image detect finished'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                # 在命令行中显示图像检测结果
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


        # 9. 'SCREEN_RETURN'嵌套状态  ==>  'RECOGNITION2'
        # @描述：图像识别返回机械臂操作指令
        # @输入：                        全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:       ud_operation_info --> observe_input / image_input
        #       手臂之前所在观测位置:          ud_current_pose2  --> current_pose
        # @输出:                            局部userdata       --> 全局userdata
        #       手臂之后所在观测位置:          current_pose      --> ud_current_pose2
        #       发给机械臂执行的正则数组:       image_return      --> ud_operation_info
        image_detect_return = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['ud_operation_info', 'ud_current_pose2'], 
            output_keys=['ud_current_pose2', 'ud_operation_info'])
        with image_detect_return:
            # 9.1 'OBSERVE_POSITION'子状态  ==>  'DETECT_IMAGE_RETURN'
            # @描述：若需要检测2号面板数码管，则2号机械臂需要运动到观测位置
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:   ud_operation_info --> observe_input
            #       手臂之前所在观测位置:      ud_current_pose2  --> current_pose
            # @输出:                        局部userdata       --> 全局userdata
            #       手臂运动后所在观测位置：   current_pose      --> ud_current_pose2
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
                # 5、18表示是图像流，机械臂仍在原位
                if userdata.observe_input[1] == 5 or userdata.observe_input[1] == 18:
                    userdata.current_pose = userdata.current_pose
                # 否则是2号面板，认为机械臂已到位
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
                transitions = {'succeeded':'DETECT_IMAGE_RETURN'},
                remapping = {'observe_input':'ud_operation_info','current_pose':'ud_current_pose2'})


            # 9.2 'DETECT_IMAGE_RETURN'子状态
            # @描述：调用显示屏检测模块，识别2号面板的时间，并将数字按动顺序转为正则数组，发给机械臂执行
            # @输入：                       全局userdata       --> 局部userdata
            #       任务管理器下发的正则数组:   ud_operation_info --> image_input
            # @输出:                        局部userdata       --> 全局userdata
            #       发给机械臂执行的正则数组:   image_return      --> ud_operation_info
            @smach.cb_interface(input_keys=['image_input'])
            def image_request_cb(userdata, request):
                actionFeedback.smach_info = 'image detect'
                actionServer.publish_feedback(actionFeedback)
                process_info.publish(actionFeedback.smach_info)

                image_request = ScreenDetectReturnRequest()
                image_request.param_b = int(userdata.image_input[0])#b
                image_request.param_c = int(userdata.image_input[1])#c
                image_request.param_d = userdata.image_input[2]     #d
                image_request.param_e = int(userdata.image_input[3])#e
                image_request.param_f = int(userdata.image_input[4])#f
                image_request.param_g = int(userdata.image_input[5])#g
                # 检查发送的消息是否正确
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

                # 12:12:00 加3分钟，从而转化为1,2,1,5
                a1 = int(response.time[0]) / 10 
                a2 = int(response.time[0]) % 10
                b1 = (int(response.time[1])+3) / 10
                b2 = (int(response.time[1])+3) % 10
                # 2表示2号面板，4表示'连续按钮'操作类型，6表示按动6个数字，自动补充秒数00
                userdata.image_return = [2, 4, 6, number[a1], number[a2], number[b1], number[b2], number[0], number[0]] 
                return
            StateMachine.add('DETECT_IMAGE_RETURN',
                ServiceState('/halcon_detection_return_service', ScreenDetectReturn,
                    request_cb=image_request_cb,
                    response_cb=image_response_cb),
                transitions = {'succeeded':'succeeded'},
                remapping = {'image_input':'ud_operation_info','image_return':'ud_operation_info'})
        StateMachine.add('SCREEN_RETURN', image_detect_return, {'succeeded':'RECOGNITION2'})


        # 10. 'LIGHT_DETECTION'状态  ==>  'TASK_MANAGER'
        # @描述：调用状态检测模块，识别按钮灯光
        # @输入：                        全局userdata       --> 局部userdata
        #       任务管理器下发的正则数组:   ud_operation_info --> light_input
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

            # 判断识别出的灯光状态是否与期望的相同
            if response.state ==  int(userdata.light_input[3]):
                rospy.logwarn("light result is: ")
                rospy.logwarn(userdata.light_input[3])
            else:
                rospy.logerr("light result is not: ")
                rospy.logerr(userdata.light_input[3])
            return
        StateMachine.add('LIGHT_DETECTION',
            ServiceState('/svm', ButtonDetectState,
                request_cb=svm_request_cb,
                response_cb=svm_response_cb),
            transitions = {'succeeded':'TASK_MANAGER'},
            remapping = {'light_input':'ud_operation_info'})


        # 11. 'RETURN_TO_ORIGIN1'状态  ==>  'TASK_MANAGER'
        # @描述：调用机械臂移动模块，1号右臂归位
        # @输入：                   全局userdata      --> 局部userdata
        #       手臂之前所在观测位置: ud_current_pose1 --> current_pose
        # @输出:                   局部userdata      --> 全局userdata
        #       手臂之后所在观测位置: current_pose     --> ud_current_pose1
        @smach.cb_interface(input_keys=['current_pose'])
        def observe_goal_cb(userdata, goal):  
            actionFeedback.smach_info = 'go to origin pose 18 started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)
            
            observe_goal = ButtonObserveMoveGoal()
            # 每次动完1号机械臂就返回18中间点
            observe_goal.button_index = [userdata.current_pose, 18]
            return observe_goal
        @smach.cb_interface(output_keys=['current_pose'])
        def observe_result_cb(userdata, status, result): 
            # 认为1号机械臂已经到18号中间点
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


        # 12. 'RETURN_TO_ORIGIN2'状态  ==>  succeeded，结束！
        # @描述：调用机械臂移动模块，2号左臂归位
        # @输入：                   全局userdata      --> 局部userdata
        #       手臂之前所在观测位置: ud_current_pose2 --> current_pose
        # @输出:                   局部userdata      --> 全局userdata
        #       手臂之后所在观测位置: current_pose     --> ud_current_pose2
        @smach.cb_interface(input_keys=['current_pose'])
        def observe_goal_cb(userdata, goal): 
            actionFeedback.smach_info = 'go to origin pose 11 started'
            actionServer.publish_feedback(actionFeedback)
            process_info.publish(actionFeedback.smach_info)

            observe_goal = ButtonObserveMoveGoal()
            # 2号机械臂返回11号面板观测位置
            observe_goal.button_index = [userdata.current_pose, 11]
            return observe_goal
        @smach.cb_interface(output_keys=['current_pose'])
        def observe_result_cb(userdata, status, result): 
            # 认为2号机械臂已经到11号观测位置
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

    # Attach a SMACH introspection server, 可用rosrun smach_viewer smach_viewer.py查看状态机流程图（需要python2环境）
    sis = IntrospectionServer('smach_demo_1', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    # 全部任务未执行完毕: 循环等待TaskServiceCB类中的任务执行完毕
    while not central_control_action_result:
        sleep(0.1)
    # 全部任务执行完毕, action返回true
    actionServer.set_succeeded(actionResult)
    
# 主程序
# =================================================================
def main():
    rospy.init_node('hangdian_state_machine_wuhu')

    # 全局action server，execute回调函数在此
    global actionServer
    actionServer = actionlib.SimpleActionServer('central_control', CentralControlRunAction, execute, auto_start=False)
    actionServer.start()

    rospy.loginfo('central control server started')

    rospy.spin()

if __name__ == '__main__':
    main()
