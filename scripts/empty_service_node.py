#!/usr/bin/env python2


import std_srvs.srv
import rospy
import random

def handle_request(req):
    print("Returning")
    index = random.randint(1,3)
    # index = 3
    if index == 1:
        print("this is button")
        rospy.sleep(1)
        return [True,"1"]
    elif index == 2:
        print("this is lever")
        rospy.sleep(1)
        return [True,"2"]
    elif index == 3:
        print("this is screen")
        rospy.sleep(1)
        return [True,"3"]
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