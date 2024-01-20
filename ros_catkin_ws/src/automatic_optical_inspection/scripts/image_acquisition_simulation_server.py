#!/usr/bin/env python


"""
    This script is an image acquisition simulation node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Simulation 
    node return requested image from local assets and provide an option for development without 
    the need of camera.
"""

import sys
import cv2
import rospy
from cv_bridge import CvBridge

from core.models import test
from automatic_optical_inspection.srv import ImageAcquisition

node_name = "image acquisition simulation"


def handle_image_acquisition(req):
    rospy.loginfo("Image requested")

    im = cv2.imread(
        "/media/egor/T7/vut_dp_project_workspace/assets/datasets/fpic-component-dataset/fpic-component/val/img/image_3.png"
    )

    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(im)


if __name__ == "__main__":
    rospy.init_node("image_acquisition_simulation")
    rospy.loginfo(f"{node_name.capitalize()} is alive!")
    rospy.loginfo(sys.executable)
    s = rospy.Service("acquire_image", ImageAcquisition, handle_image_acquisition)
    rospy.loginfo(f"Spinning {node_name}...")
    rospy.spin()
