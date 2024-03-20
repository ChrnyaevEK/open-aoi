"""
    This script is a definition of moderator node. Node control communication of other nodes with each other.
"""

import pickle

import rclpy
import numpy as np

from rclpy.node import Node


from open_aoi_interfaces.srv import ServiceStatus

NODE_NAME = "mediator"
BLOB_STORAGE_DIR = "./assets/storage"

IMAGE_ACQUISITION_NODE = "image_acquisition"


class Service(Node):
    service_status_default: str = "Working"
    service_status: str = service_status_default

    def __init__(self):
        super().__init__(NODE_NAME)
        # --- Services ---
        self.inspection_trigger_service = self.create_service(
            None,
            f"{NODE_NAME}/inspection/trigger",
            None,
        )

        self.status_service = self.create_service(
            ServiceStatus,
            f"{NODE_NAME}/status",
            self.expose_status,
        )

        self.health_service = self.create_service(
            ServiceStatus,
            f"{NODE_NAME}/health",
            self.expose_status,
        )

        self.logger = self.get_logger()

    def inspection_trigger(self, request_class, response_class):
        pass

    def health(self, request_class, response_class):
        pass

    def status(self, request_class, response_class):
        return ServiceStatus.Response(status=self.service_status)


def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
