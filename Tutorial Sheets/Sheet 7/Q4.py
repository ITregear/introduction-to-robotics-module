# Author: ivantregear
# Date Created: 24/02/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2


def main():
    vidcap = cv2.VideoCapture('CarsMoving.mp4')
    success, image = vidcap.read()

    sigma = 1

    last_background = np.zeros_like(image, dtype='int8')
    background = np.zeros_like(image, dtype='int8')

    while success:
        success, image = vidcap.read()

        if success:
            background = last_background + np.maximum(np.minimum(image - last_background, sigma), -sigma)
            last_background = background

            cv2.imshow("Background", background)
            cv2.imshow("Video", image)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
