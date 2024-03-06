import cv2 as cv
import numpy as np


def cut(im: np.ndarray, cv_stat_value):
    # Function parse CV connected component detection statics (values)
    # to cut out component from provided image
    t = cv_stat_value[cv.CC_STAT_TOP]
    l = cv_stat_value[cv.CC_STAT_LEFT]

    w = cv_stat_value[cv.CC_STAT_WIDTH]
    h = cv_stat_value[cv.CC_STAT_HEIGHT]

    return im[t : t + h, l : l + w]


def align(im, template):
    # Function perform alingment of two images using ORB algorithm (open source alternative for SURF)

    # Use ORB to detect keypoints and extract (binary) local
    # invariant features
    orb = cv.ORB_create(1000)

    (kpsA, descsA) = orb.detectAndCompute(im, None)
    (kpsB, descsB) = orb.detectAndCompute(template, None)

    # Match the features
    method = cv.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING
    matcher = cv.DescriptorMatcher_create(method)
    matches = matcher.match(descsA, descsB, None)

    # Sort the matches by their distance (the smaller the distance,
    # the "more similar" the features are)
    matches = sorted(matches, key=lambda x: x.distance)
    # keep only the top matches
    keep = int(len(matches) * 0.5)
    matches = matches[:keep]

    # Check to see if we should visualize the matched keypoints
    # matchedVis = cv.drawMatches(im, kpsA, template, kpsB, matches, None)
    # matchedVis = imutils.resize(matchedVis, width=1000)
    # imshow(matchedVis)

    # Allocate memory for the keypoints (x, y)-coordinates from the
    # top matches -- we'll use these coordinates to compute our
    # homography matrix
    pts_a = np.zeros((len(matches), 2), dtype="float")
    pts_b = np.zeros((len(matches), 2), dtype="float")
    # Loop over the top matches
    for i, m in enumerate(matches):
        # indicate that the two keypoints in the respective images
        # map to each other
        pts_a[i] = kpsA[m.queryIdx].pt
        pts_b[i] = kpsB[m.trainIdx].pt

    # Compute the homography matrix between the two sets of matched
    # points
    (H, mask) = cv.findHomography(pts_a, pts_b, method=cv.RANSAC)
    # Use the homography matrix to align the images
    (h, w) = template.shape[:2]
    im = cv.warpPerspective(im, H, (w, h))
    # Return the aligned image
    return im