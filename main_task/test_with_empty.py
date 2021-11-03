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

from numpy.core.numeric import require
import rospy
import threading

import smach
from smach import StateMachine, State
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

import std_srvs.srv
from geometry_msgs.msg import Pose
# 需要source才能找到自定义的msg！！
from hangdian_msgs.srv import ButtonDetectPose, ButtonDetectPoseRequest, ButtonDetectPoseResponse, ButtonDetectState, ButtonDetectStateRequest, ButtonDetectStateResponse
from hangdian_msgs.srv import ScreenDetect, ScreenDetectRequest, TaskManage, TaskManageResponse
from hangdian_msgs.msg import LeverDetectAction, LeverDetectGoal, ButtonManipulateAction, ButtonManipulateGoal

#std_srvs.srv.SetBoolRequest
# from iiwa_msgs.msg import ControlMode
# from iiwa_msgs.srv import ConfigureControlMode
# import iiwa_msgs.msg

class TaskServiceCB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'lever', 'button', 'screen'], output_keys=['index_of_button'])
        self.outcome = 'succeeded'
    def execute(self, userdata):
        self.outcome= 'succeeded'
        rospy.wait_for_service('/hangdian/task_manager_service')
        try:
            get_button_index = rospy.ServiceProxy('/hangdian/task_manager_service', TaskManage)
            response = get_button_index()
            have_task = response.have_task
            if not have_task:
                self.outcome = 'succeeded'
            else:
                userdata.index_of_button = response.module
                if response.module == 1:
                    print("To detect button !!!")
                    self.outcome = 'button'
                elif response.module == 2:
                    print("To detect lever !!!")
                    self.outcome = 'lever'
                elif response.module == 3:
                    print("To detect screen !!!")
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
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger),
                {'succeeded':'TASK_MANAGER', 'aborted':'aborted'})


        # 2. 任务管理器
        ## 如果任务未完成，继续检测
        ## 任务完成，则到总sm的succeed状态
        # check if sensors are OK
        StateMachine.add('TASK_MANAGER',
            TaskServiceCB(),
            transitions = {'succeeded':'succeeded', 'lever':'DETECT_LEVER', 'button':'RECOGNITION','screen':'IMAGE_DETECT'},
            remapping = {'index_of_button':'ud_index_of_button'})
        # smach_ros里的ServiceState只有succeeded、aborted、preempted三种状态

        # 3. 操纵杆检测
        # check if sensors are OK
        
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
            ServiceState('hangdian/smach/empty_service_req', std_srvs.srv.SetBool,
                request = lever_req,
                response_cb=lever_response_cb),
            transitions = {'succeeded':'TASK_MANAGER'},# succeeded should be TASK_M in real
            remapping = {'lever_input':'ud_index_of_button', 'result':'final_result'} # userdata in viewer are only ud_xxx and final_xxx
            )
            


        # 4. 视觉识别（嵌套状态机）
        button_detect = StateMachine(outcomes=['succeeded','aborted','preempted'])
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
            

            ## 4.2 YOLO-ICP
            poseDetect = ButtonDetectPoseRequest()
            poseDetect.button_index = '1'
            # yolo icp for button localization
            def yolo_response_cb(userdata, response):
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('YOLOICP',
                ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                    response_cb=yolo_response_cb),
                transitions={'succeeded':'SVM'})
            
            ## 4.3 状态识别（也许可以和4.2并行？）
            
            def svm_response_cb(userdata, response):
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('SVM',
                ServiceState('hangdian/smach/empty_service',std_srvs.srv.Trigger,
                    response_cb=svm_response_cb),
                {'succeeded':'succeeded'})
        StateMachine.add('RECOGNITION',button_detect, {'succeeded':'ARM_DETECT'})
        
        
        # 5. 机械臂检测(大action)
        def arm_response_cb(userdata, response):
            if response.success == True:
                return
            else:
                return
        StateMachine.add('ARM_DETECT', 
            ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                response_cb=arm_response_cb), 
            transitions = {'succeeded':'TASK_MANAGER'})# succeeded should be TASK_M in real
        

        # 6. 图像识别（嵌套状态机，操作后即可并行）
        image_detect = StateMachine(outcomes=['succeeded','aborted','preempted'])
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
            

            ## 6.2 图像识别
            ## 成功--> 回到2. 任务管理器
            # arm go to observe position
            
            def image_response_cb(userdata, response):
                if response.success == True:
                    return
                else:
                    return
            StateMachine.add('DETECT_IMAGE',
                ServiceState('hangdian/smach/empty_service', std_srvs.srv.Trigger,
                    response_cb=image_response_cb),
                {'succeeded':'succeeded'})
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
