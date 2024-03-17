"""
    This script is an image acquisition node. See service definition for request format.
    Request format is mutual for simulation and real node (this nodes are interchangeable). Node
    provide interface to real camera hiding hardware set up and communication routines. 
"""

import os
import pickle
from typing import List, Optional, Type

import rclpy
import numpy as np
from pypylon import pylon
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image


from open_aoi_interfaces.srv import ImageAcquisition, ServiceStatus

NODE_NAME = "image_acquisition"
EMULATION_DIR = "blob_storage/cameras/emulation"


class Service(Node):
    camera_ip_address: str = ""
    camera_exposure_us: int = 4000
    camera_emulation_mode: bool = True

    camera: Optional[pylon.InstantCamera] = None
    service_status_default: str = "Working"
    service_status: str = service_status_default

    def __init__(self):
        super().__init__(NODE_NAME)
        # --- Services ---
        self.acquire_image_service = self.create_service(
            ImageAcquisition,
            f"{NODE_NAME}/acquire_image",
            self.acquire_image,
        )

        self.status_service = self.create_service(
            ServiceStatus,
            f"{NODE_NAME}/expose_status",
            self.expose_status,
        )

        # --- Parameters ---
        self.declare_parameter(
            "camera_emulation_mode",
            value=self.camera_emulation_mode,
            descriptor=ParameterDescriptor(
                name="Camera emulation mode",
                type=rclpy.Parameter.Type.BOOL.value,
                description="If True, camera emulation is used. False by default.",
            ),
        )

        self.declare_parameter(
            "camera_ip_address",
            value=self.camera_ip_address,
            descriptor=ParameterDescriptor(
                name="Camera IP address",
                type=rclpy.Parameter.Type.STRING.value,
                description="IP address of camera to use. If not provided, node will connect to the first found camera.",
            ),
        )

        self.declare_parameter(
            "camera_exposure_us",
            value=self.camera_exposure_us,
            descriptor=ParameterDescriptor(
                name="Camera exposure time [us]",
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Camera exposure time in microseconds. Not applied in emulation mode.",
            ),
        )
        self.add_on_set_parameters_callback(self._update_parameters)
        self.logger = self.get_logger()

        # Startup routine
        self._reload_service()

    def _update_parameters(self, parameters: List[rclpy.Parameter]):
        for p in parameters:
            print(p.name)
            setattr(self, p.name, p.value)
        # Call service setup functions after update
        self._reload_service()
        return SetParametersResult(successful=True, reason="")

    def _reload_service(self):
        try:
            # 1. Reacquire camera
            self._acquire_camera()
        except RuntimeError as e:
            self.service_status = str(e)
            return

    def _acquire_camera(self):
        if self.camera is not None:
            self.camera.Close()

        if self.camera_emulation_mode:
            os.environ["PYLON_CAMEMU"] = "1"
            tlf: pylon.TlFactory = pylon.TlFactory.GetInstance()
            self.camera = pylon.InstantCamera(tlf.CreateFirstDevice())
            self.camera.Open()
            self.camera.ImageFilename = EMULATION_DIR
            self.camera.ImageFileMode = "On"
            self.camera.TestImageSelector = "Off"
            self.camera.Height = 2048
            self.camera.Width = 2592
            self.service_status = "Connected to camera: EMULATION"
        else:
            tlf: pylon.TlFactory = pylon.TlFactory.GetInstance()
            for dev_info in tlf.EnumerateDevices():
                if (
                    dev_info.GetDeviceClass() == "BaslerGigE"
                    and dev_info.GetIpAddress() == self.camera_ip_address
                ):
                    self.camera = pylon.InstantCamera(tlf.CreateDevice(dev_info))
                    break
            else:
                raise RuntimeError(
                    f"Failed to acquire camera with IP: {self.camera_ip_address}"
                )
            self.camera.Open()
            self.camera.ExposureTime = self.camera_exposure_us
            self.service_status = f"Connected to camera: {self.camera_ip_address}"

    def _image_to_message(self, im: np.ndarray):
        msg = Image()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "0"

        msg.encoding = "bgr8"
        msg.height, msg.width = im.shape[:2]
        msg.step = msg.width * 3

        msg.data = pickle.dumps(im)

        return msg

    def expose_status(self, request_class, response_class):
        return ServiceStatus.Response(status=self.service_status)

    def acquire_image(self, request_class, response_class):
        self.logger.info("Image requested")

        default_image = np.zeros((1, 1, 1))

        if self.camera is None:
            return ImageAcquisition.Response(
                image=self._image_to_message(default_image),
                error="CAMERA_GENERAL",
                error_description="Capture image called before camera initialization",
            )

        grab_result = self.camera.GrabOne(1000)
        if grab_result.GrabSucceeded():
            # Access the image data
            image = grab_result.Array
            return ImageAcquisition.Response(
                image=self._image_to_message(image),
                error="NONE",
                error_description="",
            )
        else:
            print("Error: ", grab_result.ErrorCode, grab_result.ErrorDescription)
        grab_result.Release()


def main(args=None):
    rclpy.init(args=args)
    service = Service()

    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
