#!/usr/bin/env python

"""
    This script is an image acquisition node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Node
    provide interface to real camera hiding hardware set up and communication routines. 
"""

import rospy
from automatic_optical_inspection.srv import ImageAcquisition

node_name = "image acquisition"


def handle_image_acquisition(req):
    rospy.loginfo("Real image requested")
    raise NotImplementedError()


if __name__ == "__main__":
    rospy.loginfo(f"{node_name.capitalize()} is alive!")
    rospy.init_node("image_acquisition")
    s = rospy.Service("acquire_image", ImageAcquisition, handle_image_acquisition)
    rospy.loginfo(f"Spinning {node_name}...")
    rospy.spin()
