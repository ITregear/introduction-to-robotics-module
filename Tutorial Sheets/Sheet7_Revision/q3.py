import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.interpolate import interp1d


def rgb2gray(rgb):
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = gray.astype(int)
    return gray


def main():
    """
    Read image and determine if over or underexposed
    Then correct this by evenly redistributing levels of grey across image
    :return:
    """

    fig, ax = plt.subplots(2, 2)

    img = mpimg.imread('images/Tiger.jpg')

    img_grey = rgb2gray(img)

    # Counting grey values to create histogram
    grey_values = np.linspace(0, 255, 256)
    count = np.zeros(256)

    for j in range(len(img_grey)):
        for i in range(len(img_grey[0])):
            count[img_grey[j, i]] += 1

    # Now evenly redistributing grey values
    cumulative_count = np.cumsum(count)
    normalised_cumulative_count = cumulative_count / np.max(cumulative_count)

    # create an interpolation function
    interp_func = interp1d(grey_values, normalised_cumulative_count)

    # create new grey values for each pixel using the interpolation function
    img_adjusted = (interp_func(img_grey.flatten()).reshape(img_grey.shape)*255).astype(int)

    # Creating new histogram
    grey_values = np.linspace(0, 255, 256)
    count_adjusted = np.zeros(256)

    for j in range(len(img_grey)):
        for i in range(len(img_grey[0])):
            count_adjusted[img_adjusted[j, i]] += 1

    ax[0, 0].imshow(img_grey, cmap='gray')
    ax[0, 1].bar(grey_values, count)
    ax[1, 0].imshow(img_adjusted, cmap='gray')
    ax[1, 1].bar(grey_values, count_adjusted)

    plt.show()


if __name__ == "__main__":
    main()
