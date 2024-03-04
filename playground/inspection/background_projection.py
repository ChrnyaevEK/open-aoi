from __future__ import print_function
from __future__ import division
import cv2 as cv
import numpy as np
import argparse


def Hist_and_Backproj(val):

    bins = val
    histSize = max(bins, 2)
    ranges = [0, 180]  # hue_range

    hist = cv.calcHist([hue], [0], None, [histSize], ranges, accumulate=False)
    cv.normalize(hist, hist, alpha=0, beta=255, norm_type=cv.NORM_MINMAX)

    backproj = cv.calcBackProject([hue], [0], hist, ranges, scale=1)
    cv.imshow(window_image, backproj)


parser = argparse.ArgumentParser(description="Code for Back Projection tutorial.")
parser.add_argument(
    "--input",
    help="Path to input image.",
    default="/home/egor/Downloads/Image__2024-02-27__10-15-37.bmp",
)
args = parser.parse_args()
src = cv.imread(cv.samples.findFile(args.input))
src = cv.medianBlur(src, 7)

if src is None:
    print("Could not open or find the image:", args.input)
    exit(0)
hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
ch = (0, 0)
hue = np.empty(hsv.shape, hsv.dtype)
cv.mixChannels([hsv], [hue], ch)
window_image = "Source image"
cv.namedWindow(window_image)
bins = 25
cv.createTrackbar("* Hue  bins: ", window_image, bins, 180, Hist_and_Backproj)
Hist_and_Backproj(bins)
cv.waitKey()