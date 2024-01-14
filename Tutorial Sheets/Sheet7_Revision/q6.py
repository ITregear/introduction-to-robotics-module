# Author: ivantregear
# Date Created: 18/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import scipy.signal as sig


def rgb2gray(rgb):
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = gray.astype(int)
    return gray


def main():
    """
    Extract text from within image
    Achieved by differentiating in both direction to detect edges
    Can use convolution with a differentiating kernel
    :return:
    """

    fig, ax = plt.subplots(1, 2)

    img = mpimg.imread("images/StreetSign.jpg")
    img_grey = rgb2gray(img)

    k_d = np.array([[0.5, 0, -0.5]])

    img_dh = sig.convolve2d(img_grey, k_d, mode='same')
    img_dv = sig.convolve2d(img_grey, k_d.T, mode='same')

    img_edges = 1 - np.sqrt(img_dh**2 + img_dv**2)

    ax[0].imshow(img_grey, cmap='gray')
    ax[1].imshow(img_edges, cmap='gray')

    plt.show()


if __name__ == "__main__":
    main()
