# Author: ivantregear
# Date Created: 18/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy import ndimage


def rgb2gray(rgb):
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = gray.astype(int)
    return gray


def main():

    """
    Extract image of specific shape by applying erosion and dilation in sequence
    Kernal is a circle shape
    :return:
    """

    fig, ax = plt.subplots(2, 2)

    img = mpimg.imread('images/Circuit.jpg')

    img_grey = rgb2gray(img)

    r = 5  # Circle radius, pixels

    k = (np.array([[1 if (i-r)**2 + (j-r)**2 <= r**2 else 0 for j in range(2*r+1)] for i in range(2*r+1)]) * 255).astype(int)

    img_eroded = 255 - abs(ndimage.grey_erosion(img_grey, structure=k))

    img_dilated = ndimage.grey_dilation(img_eroded, structure=k)

    ax[0, 0].imshow(img_grey)
    ax[0, 1].imshow(k)
    ax[1, 0].imshow(img_eroded)
    ax[1, 1].imshow(img_dilated)
    plt.show()


if __name__ == "__main__":
    main()
