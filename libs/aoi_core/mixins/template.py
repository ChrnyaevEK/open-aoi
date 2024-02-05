import numpy as np


class Mixin:
    image: np.ndarray

    def assign_image(self, im: np.ndarray) -> str:
        raise NotImplemented()

    def load_image(self):
        raise NotImplemented()

    def inpaint_zones(self, im: np.ndarray) -> np.ndarray:
        raise NotImplemented()

    def crop_zones(self, im: np.ndarray) -> list[np.ndarray]:
        raise NotImplemented()
