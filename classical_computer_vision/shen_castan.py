import cv2
import numpy as np

# Constant Values, can impact the resulting image
s_factor = 0.9 # Smoothing factor 0 < sf < 1. Higher values make a "fuzzier" image
window_size = 6 # For computing the adaptive gradient
outline = 25 # For ignoring the edge's of the image
low_thresh, high_thresh = 2, 2 # Edge threshold
do_hysteresis = False
thinFactor = 0

# Helper Functions
def isEdge (row, col, rows, columns):
    if row < outline or row >= rows-outline or col < outline or col >= columns-outline:
        return True
    return False

# Takes an image (numpy ndarray) and returns an ndarray with ISEF applied
def getISEF (img):
    casual = np.zeros_like (img)
    anti_casual = np.zeros_like (img)
    first_pass = np.zeros_like (img)
    rows, columns = img.shape

    b1 = (1.0 - s_factor) / (1.0 + s_factor)
    b2 = s_factor * b1

    #################################
    # First compute for rows
    #################################

    # Compute boundaries for first and last column
    for col in range (columns):
        casual[0][col] = b1 * img[0][col]
        anti_casual[rows-1][col] = b2 * img[rows-1][col]

    # Compute the casual component
    for row in range (1, rows):
        for col in range (columns):
            casual[row][col] = b1 * img[row][col] + s_factor * casual[row-1][col]

    # Compute the anti-casual component
    for row in range (rows-2, -1, -1):
        for col in range (columns):
            anti_casual[row][col] = b2 * img[row][col] + s_factor * anti_casual[row+1][col];

    # Boundary case
    for col in range (columns-1):
        first_pass[rows-1][col] = casual[rows-1][col];

    # Store the sum of casual and anti-casual components in the result
    for row in range (rows-2):
        for col in range (columns-1):
            first_pass[row][col] = casual[row][col] + anti_casual[row+1][col]

    #################################
    # Second compute for columns
    #################################

    result = first_pass.copy () # Creates a deep copy so we can maintain our first pass results

    # Compute boundary conditions
    for row in range (rows):
        casual[row][0] = b1 * first_pass[row][0];
        anti_casual[row][columns-1] = b2 * first_pass[row][columns-1];

    # Compute the casual component
    for col in range (1, columns):
        for row in range (rows):
            casual[row][col] = b1 * first_pass[row][col] + s_factor * casual[row][col-1]

    # Compute the anti-casual component
    for col in range (columns-2, -1, -1):
        for row in range (rows):
            anti_casual[row][col] = b2 * first_pass[row][col] + s_factor * anti_casual[row][col+1];

    # Boundary case
    for row in range (rows):
        result[row][columns-1] = casual[row][columns-1];

    # Final computation, sum casual and anti-casual and store in result
    for row in range (rows):
        for col in range (columns-1):
            result[row][col] = casual[row][col] + anti_casual[row][col+1];

    return result

# Note, this could be sped up with parallelism in multiple ways. Since image have constant sizes, you could break the image
#  into x chunks and use x threads to process.
#  ISEF works in two stages, with the second stage relying on the work of the first stage. If there was a thread queue the first
#  stage could push it's work onto the queue while another thread processes the incoming work. This would optimization will be
#  less than 2x, but it could be usefull to think about.


def getBLI (img, imgISEF):
    rows, columns = img.shape
    result = np.zeros_like (img)

    for row in range (rows):
        for col in range (columns):
            result[row][col] = float ((imgISEF[row][col] - img[row][col]) > 0.0); # The inner statement returns bool, cast to float
    return result

def isEdgeCandidate (imgBLI, imgISEF, row, col):
    # Positive z - c (row)
    if imgBLI[row][col] == 1.0 and imgBLI[row+1][col] == 0.0:
        return True if (imgISEF[row+1][col] - imgISEF[row-1][col] > 0.0) else False

    # Positive z - c (col)
    elif imgBLI[row][col] == 1.0 and imgBLI[row][col+1] == 0.0:
        return True if (imgISEF[row][col+1] - imgISEF[row][col-1] > 0.0) else False

    # Negative z - c (row)
    elif imgBLI[row][col] == 1.0 and imgBLI[row-1][col] == 0.0:
        return True if (imgISEF[row+1][col] - imgISEF[row-1][col] < 0.0) else False

    # Negative z - c (col)
    elif imgBLI[row][col] == 1.0 and imgBLI[row][col-1] == 0.0:
        return True if (imgISEF[row][col+1] - imgISEF[row][col-1] < 0.0) else False

    return False

def adaptiveGradient (imgBLI, imgISEF, row, col):
    sum_on, sum_off = 0.0, 0.0
    num_on, num_off, avg_on, avg_off = 0, 0, 0, 0
    window_start, window_end = int(-window_size/2), int(window_size/2)

    for i in range (window_start, window_end):
        for j in range (window_start, window_end):
            if imgBLI[row+i][col+j] > 0.0:
                sum_on += imgISEF[row+i][col+j]
                num_on += 1
            else:
                sum_off += imgISEF[row+i][col+j]
                num_off += 1

    avg_off = sum_off / num_off if (sum_off > 0.0) else 0
    avg_on = sum_on / num_on if (sum_on > 0.0) else 0

    return avg_off - avg_on

def locateZc (imgISEF, imgBLI):
    rows, columns = imgISEF.shape
    result = np.zeros_like (imgISEF)
    for row in range (rows):
        for col in range (columns):
            if isEdge (row, col, rows, columns):
                continue
            # If there is an edge candidate here then compute the adaptive gradient (shen&castan), else set 0.0
            result[row][col] = adaptiveGradient (imgBLI, imgISEF, row, col) if isEdgeCandidate (imgBLI, imgISEF, row, col) else 0.0
    return result

# This function is used to mark connections, this actually "draws" the lines
edges = None
def mark_connected (row, col, level, img):
    global edges
    if edges[row][col] != 0:
        return 0
    if img[row][col] == 0.0:
        return 0
    if img[row][col] > low_thresh:
        edges[row][col] = 1
    else:
        edges[row][col] = 255

    notChainEnd = 0
    notChainEnd |= mark_connected(row, col+1, level+1, img);
    notChainEnd |= mark_connected(row, col-1, level+1, img);
    notChainEnd |= mark_connected(row+1, col+1, level+1, img);
    notChainEnd |= mark_connected(row+1, col, level+1, img);
    notChainEnd |= mark_connected(row+1, col-1, level+1, img);
    notChainEnd |= mark_connected(row-1, col-1, level+1, img);
    notChainEnd |= mark_connected(row-1, col, level+1, img);
    notChainEnd |= mark_connected(row-1, col+1, level+1, img);

    if notChainEnd and level > 0:
        if thinFactor > 0:
            if level % thinFactor != 0:
                edges[row][col] = 255

    return 1

def threshold_edges (img):
    global edges
    rows, columns = img.shape
    result = np.zeros_like (img)
    edges = result
    # low_thresh, high_thresh = estimate_thresh (rows, columns)
    if not do_hysteresis:
        low_thresh = high_thresh

    for row in range (rows):
        for col in range (columns):
            if isEdge (row, col, rows, columns):
                continue
            if img[row][col] > high_thresh:
                mark_connected (row, col, 0, img)

    for row in range (rows):
        for col in range (columns):
            if edges[row][col] == 255:
                edges[row][col] = 0;
    return edges

def shenCastan (img):
    img = img.astype (float)
    imgISEF = getISEF (img)
    imgBLI = getBLI (img, imgISEF)
    imgZC = locateZc (imgISEF, imgBLI)
    imgEdges = threshold_edges (imgZC)

    rows,columns = imgEdges.shape
    for row in range (rows):
        for col in range (columns):
            imgEdges[row][col] = 255 if imgEdges[row][col] > 0 else 0

    return imgEdges

img = cv2.imread ("simfield1.png", 0)
result = shenCastan(img)

#show image
cv2.imshow("Original", img)
cv2.waitKey(0)

#show image
cv2.imshow("Shen Castan", result)
cv2.waitKey(0)
