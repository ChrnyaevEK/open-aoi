#!/usr/bin/env python

"""
    This script is an image acquisition node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Node
    provide interface to real camera hiding hardware set up and communication routines. 
"""

import rospy
from automatic_optical_inspection.srv import ImageAcquisition, ImageAcquisitionRequest, ImageAcquisitionResponse

node_name = "image_acquisition"
fs = rospy.get_param("/file_system")

def handle_image_acquisition(req: ImageAcquisitionRequest) -> ImageAcquisitionResponse:
    rospy.loginfo("Real image requested")
    raise NotImplementedError()


if __name__ == "__main__":
    rospy.init_node(node_name)
    s = rospy.Service("image_acquisition/acquire", ImageAcquisition, handle_image_acquisition)

    rospy.loginfo(f"Spinning {node_name}...")
    rospy.loginfo(f'File system: {fs}')
    rospy.spin()
