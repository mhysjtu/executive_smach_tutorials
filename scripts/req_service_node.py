#!/usr/bin/env python2


import std_srvs.srv
import rospy

def handle_request(req):
    res = std_srvs.srv.SetBoolResponse()
    if req.data:
        rospy.sleep(2)
        print("succeed")
        res.success = True
        res.message = 'succeed'
    else:
        rospy.sleep(2)
        print("failed")
        res.success = True
        res.message = 'failed'
    return res
    

def empty_server():
    rospy.init_node('smach_empty_server_req')
    s = rospy.Service('hangdian/smach/empty_service_req', std_srvs.srv.SetBool, handle_request)
    print("empty server req is ready.")
    rospy.spin()

if __name__ == "__main__":
    empty_server()