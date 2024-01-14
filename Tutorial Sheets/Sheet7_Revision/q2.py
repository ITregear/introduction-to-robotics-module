# Author: ivantregear
# Date Created: 18/04/2023
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

    fig, ax = plt.subplots(3, 2)

    img = mpimg.imread('images/Dover.jpg')

    img_gray = rgb2gray(img)
    img_cliff = np.zeros_like(img_gray)
    img_cliff_reduced = np.zeros_like(img_gray)
    img_gray_reduced = np.zeros_like(img_gray)

    gray_count = 0
    threshold = 180

    img_inv = 255 - img_gray

    for j in range(len(img_gray)):
        for i in range(len(img_gray[0])):
            if img_gray[j, i] >= threshold:
                gray_count += 1
                img_cliff[j, i] = img_gray[j, i]
            img_cliff_reduced[j, i] = np.floor(img_cliff[j, i]/(255/4))*(255/4)
            img_gray_reduced[j, i] = np.floor(img_gray[j, i]/(255/4))*(255/4)

    threshold_percentage = gray_count / len(img_gray.flatten() * 100)

    print("Percentage of greyscale image that is above threshold of {}: {}%".format(threshold, threshold_percentage))

    ax[0, 0].imshow(img)
    ax[0, 1].imshow(img_gray, cmap='gray')
    ax[1, 0].imshow(img_cliff, cmap='gray')
    ax[1, 1].imshow(img_cliff_reduced, cmap='gray')
    ax[2, 0].imshow(img_gray_reduced, cmap='gray')
    ax[2, 1].imshow(img_inv, cmap='gray')
    plt.show()


if __name__ == "__main__":
    main()
