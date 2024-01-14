# Author: ivantregear
# Date Created: 10/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from sklearn.cluster import KMeans
from sklearn.cluster import SpectralClustering
from scipy import interpolate


def find_closest(array, value):
    return (np.abs(array - value)).argmin()


def main():
    img_name = 'img'
    img = mpimg.imread(img_name + '.png')
    fig, ax = plt.subplots(nrows=2, ncols=3)

    # Resizing image (as feature recognition does not require high resolution)
    # Making image roughly 300x300 pixels
    resizing_factor = int(len(img) / 250)  # Roughly creates 250x250 sized image
    img = img[::resizing_factor, ::resizing_factor, :]

    # Image scale needs to be adjusted so that mobile robot and RRPR manipulator can reasonably fit within boundary
    # There is no scale on the image, so one will be chosen that roughly makes the distance between B and C
    # 0.75m, as the RRPR workspace has a radius of roughly 1.5m

    # Normalizing so values are 0-255 instead of 0-1

    img = cv2.normalize(img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F).astype(np.uint8)

    if np.shape(img)[2] == 4:
        print("Removed Alpha Channel")
        img = img[:, :, :-1]
    else:
        print("No Alpha Channel to remove")

    # There are three features of interest
    # Via points are blue, end-effector target is green, and boundary is red
    # Thus binary masks can be created by thresholding each colour channel of the
    # original image separately.

    # RGB value for the three target colours was identified using a colour picker
    # Then binary arrays are true if the colour of the input image is the same
    # Within a tolerance of +/- 5 for all RGB channels

    # This creates 3 binary masks

    img_bin_red = np.zeros(np.shape(img)[:-1])
    img_bin_green = np.zeros(np.shape(img)[:-1])
    img_bin_blue = np.zeros(np.shape(img)[:-1])

    print("Thresholding to create Binary Masks")

    for i in range(len(img)):
        for j in range(len(img[0])):
            if np.isclose(img[i, j, :], np.array([220, 42, 2]), atol=10).all():
                img_bin_red[i, j] = 1
            if np.isclose(img[i, j, :], np.array([144, 255, 60]), atol=10).all():
                img_bin_green[i, j] = 1
            if np.isclose(img[i, j, :], np.array([1, 0, 251]), atol=10).all():
                img_bin_blue[i, j] = 1

        print(round(i / len(img) * 100, 1), "%")

    # One point mask has two objects in them, with need separating
    # Using a K-means clustering algorithm, these can be grouped into two bodies
    # This also conveniently provides the locations of the points (the means)

    # Following pre-processes image data into datasets that scikit algorithms can interpret
    y = np.linspace(0, len(img_bin_red) - 1, len(img_bin_red))
    x = np.linspace(0, len(img_bin_red[0]) - 1, len(img_bin_red[0]))
    X, Y = np.meshgrid(x, y)
    XY = np.array([X.flatten(), Y.flatten()]).T

    x_green = XY[img_bin_green.flatten() == 1]
    x_blue = XY[img_bin_blue.flatten() == 1]

    print("Calculating positions of blue and green points")

    km_green = KMeans(n_clusters=1)
    km_blue = KMeans(n_clusters=2)
    km_green.fit(x_green)
    km_blue.fit(x_blue)

    green_pos = km_green.cluster_centers_[0]

    blue_start = km_blue.cluster_centers_[(np.abs(km_blue.cluster_centers_ - km_green.cluster_centers_)).argmax(axis=0)[0]]
    blue_end = km_blue.cluster_centers_[(np.abs(km_blue.cluster_centers_ - km_green.cluster_centers_)).argmin(axis=0)[0]]

    pixel_to_m = 0.75 / np.sqrt((blue_end[0] - green_pos[0]) ** 2 + (blue_end[1] - green_pos[1]) ** 2)

    green_pos *= pixel_to_m
    blue_end *= pixel_to_m
    blue_start *= pixel_to_m

    # For the boundary lines, the robot will take the path defined by the locus of points equidistant from
    # both lines
    # This first requires the two lines to be segmented (although K-Means clustering is not suitable for this)
    # Then for each point in one line, the closest point from the other line will be assigned to it
    # Likely using l2 error metric
    # This creates a list of pairs of points
    # The vector middle of these point pairs defines the locus of equidistant points

    # Although somewhat convoluted for this basic example, this method should be robust for any pair of red lines
    # that acts as two boundaries, which will be demonstrated

    x_red = XY[img_bin_red.flatten() == 1]

    print("Segmenting boundaries and calculating path line")

    sc_red = SpectralClustering(2)
    y_pred = sc_red.fit_predict(x_red)

    b1_xy = np.column_stack([x_red[y_pred == 0, 0], x_red[y_pred == 0, 1]])
    b2_xy = np.column_stack([x_red[y_pred == 1, 0], x_red[y_pred == 1, 1]])

    b2_ordered = np.zeros_like(b1_xy)

    # Now b1_xy will remain the same, but b2_xy will be re-ordered so point closest to each b1_xy
    for i, row in enumerate(b1_xy):
        error = (b2_xy - row)[:, 0] ** 2 + (b2_xy - row)[:, 1] ** 2
        closest_b2 = np.argmin(error)
        b2_ordered[i] = b2_xy[closest_b2]

    path = (b1_xy + b2_ordered) / 2
    path_m = path * pixel_to_m

    b1_xy_m = b1_xy * pixel_to_m
    b2_xy_m = b2_xy * pixel_to_m

    # Path now needs smoothing, as resolution of pixels causes sudden changes in theta
    # Using spline approximation fitting interpolation

    idx = path_m[:, 0].argsort()
    x = path_m[idx, 0]
    y = path_m[idx, 1]

    tck, u = interpolate.splprep([x, y], s=1.5)  # 1.3 works well

    # evaluate the spline fits for 1000 evenly spaced distance values
    xi, yi = interpolate.splev(np.linspace(0, 1, len(path_m)), tck)

    path_m = np.column_stack([xi, yi])

    # Path now needs sorted (as clustering algorithm doesn't guarantee adjacent points are
    # next to each other

    path_m_sorted = np.zeros_like(path_m)
    path_m_sorted[0, :] = blue_start

    for i in range(1, len(path)):
        p_ref = path_m_sorted[i - 1]
        min_idx = np.argmin((path_m - p_ref)[:, 0] ** 2 + (path_m - p_ref)[:, 1] ** 2)
        path_m_sorted[i, :] = path_m[min_idx, :]
        path_m = np.delete(path_m, min_idx, axis=0)

    path_m_sorted = np.concatenate((path_m_sorted, [blue_end]), axis=0)

    print("Complete")

    plot = True
    save = True

    if save:
        with open("ImageParameters{}.txt".format(img_name), "w") as f:
            file_string = ["StartX, {}\n".format(blue_start[0]),
                           "StartY, {}\n".format(blue_start[1]),
                           "EndX, {}\n".format(blue_end[0]),
                           "EndY, {}\n".format(blue_end[1]),
                           "TargetX, {}\n".format(green_pos[0]),
                           "TargetY, {}\n\n".format(green_pos[1]),
                           "Path XY\n"]

            f.writelines(file_string)
            for row in path_m_sorted:
                f.write(str(row[0]) + "," + str(row[1]) + "\n")
        print("File Written")

    if plot:
        for axis in ax.flatten():
            axis.set_xlabel("U Pixels")
            axis.set_ylabel("V Pixels")

        ax[0, 0].set_title("Scaled and Sub-sampled Image")
        ax[0, 1].set_title("Red Mask")
        ax[0, 2].set_title("Green Mask")
        ax[1, 0].set_title("Blue Mask")
        ax[1, 1].set_title("Identified Path")
        ax[1, 2].set_title("Identified Points")
        ax[0, 0].imshow(img)
        ax[0, 1].imshow(img_bin_red)
        ax[0, 2].imshow(img_bin_green)
        ax[1, 0].imshow(img_bin_blue)
        ax[1, 1].imshow(img_bin_red)
        ax[1, 1].scatter(*zip(*b1_xy), s=1)
        ax[1, 1].scatter(*zip(*b2_xy), s=1)
        ax[1, 1].scatter(*zip(*path), s=1)
        ax[1, 2].plot(*zip(*path_m_sorted), c='r')
        ax[1, 2].scatter(green_pos[0], green_pos[1], s=12, c='green')
        ax[1, 2].scatter(blue_start[0], blue_start[1], s=12, c='blue')
        ax[1, 2].scatter(blue_end[0], blue_end[1], s=12, c='blue')
        ax[1, 2].scatter(*zip(*b1_xy_m), s=2, c='black')
        ax[1, 2].scatter(*zip(*b2_xy_m), s=2, c='black')
        ax[1, 2].set_xlim([0, None])
        ax[1, 2].set_ylim([0, None])
        ax[1, 2].set_xlabel("X Coordinate [m]")
        ax[1, 2].set_ylabel("Y Coordinate [m]")
        ax[1, 2].text(0.05, 0.95,
                      "Identified Locations:\nA: {}\nB: {}\nC: {}".format(np.round(blue_start, 3),
                                                                          np.round(blue_end, 3),
                                                                          np.round(green_pos, 3)),
                      transform=ax[1, 2].transAxes, ha='left', va='top', bbox=dict(facecolor='white', edgecolor='grey',
                                                                                   boxstyle='round,pad=0.5', alpha=0.8,
                                                                                   linewidth=0.5))
        ax[1, 2].axis('equal')
        plt.show()


if __name__ == "__main__":
    main()
