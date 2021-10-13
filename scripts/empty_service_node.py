#!/usr/bin/env python2


import std_srvs.srv
import rospy
import random

def handle_request(req):
    print("Returning")
    # index = random.randint(1,3)
    index = 1
    if index == 1:
        print("this is button")
        rospy.sleep(1)
        return [True,"button"]
    elif index == 2:
        print("this is lever")
        rospy.sleep(1)
        return [True,"lever"]
    elif index == 3:
        print("this is screen")
        rospy.sleep(1)
        return [True,"screen"]
    else :
        print("this is nothing")
        rospy.sleep(1)
        return [True,"return"]

def empty_server():
    rospy.init_node('smach_empty_server')
    s = rospy.Service('hangdian/smach/empty_service', std_srvs.srv.Trigger, handle_request)
    print("empty server is ready.")
    rospy.spin()

if __name__ == "__main__":
    empty_server()