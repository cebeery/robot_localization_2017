#!/usr/bin/env python

import rospy, sys
from nav_msgs.srv import GetMap

def get_static_map_client():
    rospy.wait_for_service('static_map') #maybe map_server
    try:
        static_map = rospy.ServiceProxy('static_map', GetMap)
        return static_map().map #return map attribute
    except rospy.ServiceException, e:
        print "Static Map Retrieve Service call failed: %s"%e


if __name__ == "__main__":

    print "Requesting Map"
    print get_static_map_client()
