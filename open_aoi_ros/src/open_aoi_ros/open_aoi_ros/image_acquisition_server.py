"""
    This script is an image acquisition node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Node
    provide interface to real camera hiding hardware set up and communication routines. 
"""

from open_aoi_interfaces.srv import ImageAcquisition

node_name = "image_acquisition"


import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__(node_name)
        self.srv = self.create_service(
            ImageAcquisition, "image_acquisition/acquire_image", self.acquire_image_callback
        )

    def acquire_image_callback(self, request, response):
        return


def main(args=None):
    rclpy.init(args=args)
    service = Service()

    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# fs = rospy.get_param("/file_system")
