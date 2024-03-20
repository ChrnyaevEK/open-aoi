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


def highlight(im: np.ndarray, cv_stat_value, value: int = 255):
    # Function acts like cut function but highlight area instead of cutting it out
    t = cv_stat_value[cv.CC_STAT_TOP]
    l = cv_stat_value[cv.CC_STAT_LEFT]

    w = cv_stat_value[cv.CC_STAT_WIDTH]
    h = cv_stat_value[cv.CC_STAT_HEIGHT]

    im = im.copy()
    im[t : t + h, l : l + w] = value
    return im


def inpaint(src: np.ndarray, trg: np.ndarray, cv_stat_value):
    # Function acts like cut function but copy area from source to target
    t = cv_stat_value[cv.CC_STAT_TOP]
    l = cv_stat_value[cv.CC_STAT_LEFT]

    w = cv_stat_value[cv.CC_STAT_WIDTH]
    h = cv_stat_value[cv.CC_STAT_HEIGHT]

    trg = trg.copy()
    trg[t : t + h, l : l + w] = src[t : t + h, l : l + w]
    return trg


def highpass(img, sigma):
    return img - cv.GaussianBlur(img, (0, 0), sigma) + 127


def extract(im: np.ndarray, color: list[int], mask: np.ndarray):
    # Function perform extraction of colored connected components from `im` according to `mask`. `color` is an RGB value.
    mask = cv.inRange(mask, color, color)
    analysis = cv.connectedComponentsWithStats(mask, cv.CV_32S)
    (_, _, values, _) = analysis

    chunks = []
    im = np.multiply(im, mask)
    for i in range(1, len(values)):
        chunks.append(255 - cut(im, values[i]))
    return chunks, values[1:]


def extract_with_mask(im: np.ndarray, mask: np.ndarray):
    # Function perform extraction of white connected components from `im` according to `mask`. `color` is an B/W image.
    analysis = cv.connectedComponentsWithStats(mask, cv.CV_32S)
    (_, _, values, _) = analysis

    chunks = []
    im = np.multiply(im, mask)
    for i in range(1, len(values)):
        chunks.append(255 - cut(im, values[i]))
    return chunks, values[1:]


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


def confusion_matrix(predictions):
    """
    Prints a confusion matrix and calculates accuracy and precision.

    Args:
        predictions: List of tuples containing (prediction, truth) pairs.

    Returns:
        None (prints the confusion matrix, accuracy, and precision)
    """
    true_positive = 0
    true_negative = 0
    false_positive = 0
    false_negative = 0

    for pred, truth in predictions:
        if pred and truth:
            true_positive += 1
        elif not pred and not truth:
            true_negative += 1
        elif pred and not truth:
            false_positive += 1
        elif not pred and truth:
            false_negative += 1

    # Calculate accuracy
    total_samples = len(predictions)
    correct_predictions = true_positive + true_negative
    accuracy = correct_predictions / total_samples

    # Calculate precision
    if true_positive + false_positive == 0:
        precision = 0
    else:
        precision = true_positive / (true_positive + false_positive)

    # Print confusion matrix
    print("Confusion Matrix:")
    print("True Positive:", true_positive)
    print("True Negative:", true_negative)
    print("False Positive:", false_positive)
    print("False Negative:", false_negative)

    # Print accuracy and precision
    print("\nAccuracy:", accuracy)
    print("Precision:", precision)


def display_truth_ratio(predictions):
    """
    Displays the ratio of 'bool truth' values in predictions.

    Args:
        predictions: List of tuples containing (prediction, truth) pairs.

    Returns:
        None (prints the ratio of 'bool truth' values)
    """
    total_samples = len(predictions)
    true_count = sum(
        truth for _, truth in predictions
    )  # Count 'True' values in the truth part of tuples

    if total_samples == 0:
        print("No predictions provided.")
    else:
        truth_ratio = true_count / total_samples
        print("Ratio of expected 'true' values:", truth_ratio)


def image_to_square_box(im: np.ndarray):
    s = max(im.shape)
    box = np.zeros((s, s))
    r = int(s / 2) - int(im.shape[0] / 2)
    c = int(s / 2) - int(im.shape[1] / 2)
    box[r : r + im.shape[0], c : c + im.shape[1]] = im
    im = box.astype(np.uint8)
    return im

def trim_zero_rows_and_cols(arr):
    # Find the index of the first non-zero element in each row
    row_sums = np.sum(arr != 0, axis=1)
    first_nonzero_row = np.argmax(row_sums > 0)

    # Find the index of the first non-zero element in each column
    col_sums = np.sum(arr != 0, axis=0)
    first_nonzero_col = np.argmax(col_sums > 0)

    # Find the index of the last non-zero element in each row
    last_nonzero_row = arr.shape[0] - 1 - np.argmax(row_sums[::-1] > 0)

    # Find the index of the last non-zero element in each column
    last_nonzero_col = arr.shape[1] - 1 - np.argmax(col_sums[::-1] > 0)

    # Slice the array to keep rows and columns between the first and last non-zero indices
    trimmed_arr = arr[
        first_nonzero_row : last_nonzero_row + 1,
        first_nonzero_col : last_nonzero_col + 1,
    ]

    return trimmed_arr


def pad_array(arr):
    # Pad 2 rows of zeros at the top and bottom
    padded_arr = np.pad(arr, ((1, 1), (1, 1)), mode="constant", constant_values=0)
    return padded_arr
