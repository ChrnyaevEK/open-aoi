#!/usr/bin/env python

"""
    This script is an image acquisition simulation node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Simulation 
    node return requested image from local assets and provide an option for development without 
    the need of camera.
"""

import os
import cv2
import rospy
import random
from cv_bridge import CvBridge

from automatic_optical_inspection.srv import ImageAcquisition, ImageAcquisitionRequest, ImageAcquisitionResponse

node_name = "image_acquisition_simulation"
fs = rospy.get_param("/file_system")


def handle_image_acquisition(req: ImageAcquisitionRequest) -> ImageAcquisitionResponse:
    assets = os.listdir(f'{fs}/assets/images')
    asset = random.choice(assets)

    rospy.loginfo(f"Simulation image requested, return: {asset}, total amount: {len(assets)}")

    im = cv2.imread(f"{fs}/assets/images/{asset}")
    im = CvBridge().cv2_to_imgmsg(im)

    return ImageAcquisitionResponse(img_data=im, img_valid=True, img_error='', img_error_description='')


if __name__ == "__main__":
    rospy.init_node(node_name)
    s = rospy.Service("image_acquisition/acquire", ImageAcquisition, handle_image_acquisition)

    rospy.loginfo(f"Spinning {node_name}...")
    rospy.loginfo(f'File system: {fs}')
    rospy.spin()
