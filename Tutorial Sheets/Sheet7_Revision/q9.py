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
    img = mpimg.imread('images/Shapes.jpg')
    img_grey = rgb2gray(img)

    fig, ax = plt.subplots(2, 2)

    # a), Classify picture into two classes -> shape or not(shape)

    # Clear that shapes are white, while not(shape) is black
    # Thus thresholding can be applied to sort data

    threshold = 150
    img_shape = img_grey > threshold

    # b), for shape class, determine pixels that are part of perimeters

    # Pixel is interior if it is surrounded on all four sides by another shape pixel
    # Will check all pixels that this applies to, and set them to 2
    # Thus 0 = not a shape
    # 1 = shape internal
    # 2 = shape perimeter

    img_shape_interior = np.zeros_like(img_shape)

    for j in range(1, len(img_shape) - 1):
        for i in range(1, len(img_shape[0]) - 1):
            if (img_shape[j + 1, i + 1] and img_shape[j + 1, i - 1]) and (img_shape[j - 1, i + 1] and img_shape[j - 1, i - 1]):
                img_shape_interior[j, i] = 1

    img_shape_perimeter = img_shape.astype(int) - img_shape_interior.astype(int)

    # c), Distinguish and establish perimeter of each individual shape

    img_identified_shapes = np.zeros_like(img_shape_perimeter)
    shape_count = 0

    for j in range(len(img_shape_perimeter)):
        for i in range(len(img_shape_perimeter[0])):
            if img_shape_perimeter[j, i]:
                # Have reached perimeter, first check if this is part of identified shape
                if not img_identified_shapes[j, i]:
                    print(img_identified_shapes[j, i])
                    shape_count += 1
                    img_identified_shapes[j, i] = shape_count
                    walking = True
                    while walking:
                        j_walk = 0
                        i_walk = 0

                        for k in range(0, 7):

                            j_1 = int(np.ceil(((k+2) % 4)/4)*np.sign(np.sin(np.pi*(k+2)/4)))
                            i_1 = int(np.ceil((k % 4)/4)*np.sign(np.sin(np.pi*k/4)))

                            if img_shape_perimeter[j+j_walk+j_1, i+i_walk+i_1] and not img_identified_shapes[j+j_walk+j_1, i+i_walk+i_1]:
                                img_identified_shapes[j+j_walk+j_1, i+i_walk+i_1] = shape_count
                                j_walk += j_1
                                i_walk += i_1
                                walking = True
                            else:
                                walking = False

    print("Number of Identified Shapes: {}".format(shape_count))

    ax[0, 0].imshow(img, cmap='gray')
    ax[0, 1].imshow(img_shape)
    ax[1, 0].imshow(img_shape_perimeter)
    ax[1, 1].imshow(img_identified_shapes)
    plt.show()


if __name__ == "__main__":
    main()
