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

    return np.floor(gray)


def main():

    fig, ax = plt.subplots(nrows=2, ncols=2)

    img = mpimg.imread('Tiger.jpg')
    img_gray = rgb2gray(img)

    values = np.linspace(0, 255, 256)
    count = np.zeros(256)

    for j in range(len(img_gray)):
        for i in range(len(img_gray[0])):
            count[int(img_gray[j, i])] += 1

    n = np.cumsum(count)
    n = n/np.max(n)
    print(np.shape(n), np.shape(img_gray[:, :]), np.shape(img.flatten()))
    img_interp = np.interp(values, n, img.flatten())

    ax[0, 0].imshow(img)
    ax[0, 1].bar(values, count)
    # ax[1, 0].imshow(img_corrected)
    # ax[1, 1].bar(values, count_corrected)
    plt.show()


if __name__ == "__main__":
    main()
