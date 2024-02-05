import numpy as np


class Mixin:
    def set_top_left_coordinates(self, x: float, y: float) -> None:
        pass

    def set_bottom_right_coordinates(self, x: float, y: float) -> None:
        pass

    def inpaint(self, im: np.ndarray) -> np.ndarray:
        pass

    def crop(self, im: np.ndarray) -> np.ndarray:
        pass
