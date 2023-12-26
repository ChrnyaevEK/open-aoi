#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge

from automatic_optical_inspection.srv import ImageAcquisition


def handle_image_acquisition(req):
    print('Random raw image requested')

    im = cv2.imread('/media/egor/T7/vut_dp_project_workspace/assets/datasets/fpic-component-dataset/fpic-component/val/img/image_3.png')

    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(im)

    


if __name__ == "__main__":
    rospy.init_node('image_acquisition_server')
    s = rospy.Service('image_acquisition', ImageAcquisition, handle_image_acquisition)
    rospy.spin()
