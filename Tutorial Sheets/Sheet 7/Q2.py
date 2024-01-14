# Author: ivantregear
# Date Created: 24/02/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def rgb2gray(rgb):

    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b

    return gray


def main():

    fig, ax = plt.subplots(nrows=3, ncols=2)

    img = mpimg.imread('Dover.jpg')
    img_gray = rgb2gray(img)
    img_threshold = np.zeros_like(img_gray)

    count = 0

    for i in range(len(img_gray)):
        for j in range(len(img_gray[0])):
            if img_gray[i, j] >= 180:
                count += 1
                img_threshold[i, j] = img_gray[i, j]

    print(count, len(img_gray)*len(img_gray[0]))
    print("Gray Percentage: {}%".format(count/(len(img_gray)*len(img_gray[0]))*100))

    img_reduced = np.floor(img_threshold/64)*64
    img_negative = ~img

    ax[0, 0].imshow(img)
    ax[0, 1].imshow(img_gray, cmap='gray')
    ax[1, 0].imshow(img_threshold, cmap='gray')
    ax[1, 1].imshow(img_reduced, cmap='gray')
    ax[2, 0].imshow(img_negative)
    plt.show()


if __name__ == "__main__":
    main()
