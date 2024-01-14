# Author: ivantregear
# Date Created: 18/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def main():

    fig, ax = plt.subplots(1, 2)

    img = mpimg.imread('images/QueenSmall.jpg')

    sf = 2
    img_size_x = img.shape[0]
    img_size_y = img.shape[1]

    img_large = img.repeat(sf, axis=0).repeat(sf, axis=1)

    ax[0].imshow(img)
    ax[1].imshow(img_large)

    plt.show()


if __name__ == "__main__":
    main()
