# Author: ivantregear
# Date Created: 24/02/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import scipy.signal as sg


def main():
    img = mpimg.imread("LadyFace.jpg")
    img_k1 = np.zeros_like(img)
    img_k2 = np.zeros_like(img)

    fig, ax = plt.subplots(nrows=1, ncols=3)

    k_1 = np.ones([21, 21]) / 21**2
    k_2 = np.zeros([7, 7])

    sigma = 7

    for j in range(len(k_2)):
        for i in range(len(k_2[0])):
            k_2[j, i] = 1/(2*np.pi*sigma**2)*np.exp(-(i**2 + j**2)/(2*sigma**2))

    img_k1[:, :, 0] = sg.convolve2d(img[:, :, 0], k_1)
    img_k2[:, :, 0] = sg.convolve2d(img[:, :, 0], k_2)



    ax[0].imshow(img)
    plt.show()


if __name__ == "__main__":
    main()
