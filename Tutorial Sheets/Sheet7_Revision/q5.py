# Author: ivantregear
# Date Created: 18/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def rgb2gray(rgb):
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = gray.astype(int)
    return gray


def main():
    """
    Smooth an image with different kernel types
    :return:
    """

    fig, ax = plt.subplots(1, 3)

    img = mpimg.imread('images/LadyFace.jpg')
    img_grey = rgb2gray(img)

    # a), Uniform kernel of size 21x21

    s = 21
    k = np.ones((s, s)) / s**2

    img_smoothed_a = sig.convolve2d(img_grey, k)

    # b), Gaussian kernel of size 7x7, and sigma=7

    sigma = 7
    [x, y] = np.meshgrid(np.arange(-7, 7+1, 1), np.arange(-7, 7+1, 1))
    k = np.exp(-(x**2+y**2)/(2*sigma**2)) / (2*np.pi*sigma**2)

    img_smoothed_b = sig.convolve2d(img_grey, k)

    ax[0].imshow(img)
    ax[1].imshow(img_smoothed_a, cmap='gray')
    ax[2].imshow(img_smoothed_b, cmap='gray')


    plt.show()


if __name__ == "__main__":
    main()
