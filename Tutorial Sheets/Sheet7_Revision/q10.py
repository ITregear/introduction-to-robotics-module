import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import scipy.signal as sig


def rgb2gray(rgb):
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = gray.astype(int)
    return gray


def main():
    """
    Detect and mathematically describe the lines contained in the image
    First need to detect edges as before by using differentiation
    Then apply Hough's transform
    :return:
    """
    fig, ax = plt.subplots(2, 2)

    img = mpimg.imread('images/Mondrian.jpg')
    img_grey = rgb2gray(img)

    k = np.array([[0.5, 0, -0.5]])  # Differentiating kernel

    img_dh = sig.convolve2d(img_grey, k, mode='same')
    img_dv = sig.convolve2d(img_grey, k.T, mode='same')

    img_lines = np.sqrt(img_dh**2 + img_dv**2)

    # Will then apply thresholding and create boolean image mask to make lines
    # more well-defined
    img_lines = img_lines > 10

    # Now applying Hough transform
    resolution = 100
    d = np.sqrt(len(img_lines)**2 + len(img_lines[0])**2)

    thetas = np.linspace(-np.pi/2, np.pi/2, resolution)
    rhos = np.linspace(-d, d, resolution)

    accumulator = np.zeros((len(rhos), len(thetas)))

    for v in range(len(img_lines)):
        for u in range(len(img_lines[0])):
            if img_lines[v, u]:
                # If pixel has non-zero value
                for theta_idx in range(len(thetas)):
                    theta = thetas[theta_idx]
                    rho = np.cos(theta) * (v + u*np.tan(theta))
                    rho_idx = np.argmin(np.abs(rhos-rho))

                    accumulator[rho_idx, theta_idx] += 1

    threshold_percentage = 75  # 80% of maximum value
    threshold_value = threshold_percentage * 0.01 * np.max(accumulator)

    threshold_idx = np.array(np.where(accumulator > threshold_value))

    m = []
    c = []

    for idx in threshold_idx.T:
        rho = rhos[idx[0]]
        theta = thetas[idx[1]]

        m += [-np.tan(theta)]
        c += [rho / np.cos(theta)]

    u_range = np.linspace(0, len(img_lines[0]), 100)

    for i in range(len(m)):
        ax[1, 1].plot(u_range, m[i]*u_range + c[i], c='red')

    ax[0, 0].imshow(img)
    ax[0, 1].imshow(img_lines)
    ax[1, 0].contourf(thetas, rhos, accumulator)
    ax[1, 1].imshow(img)
    plt.show()


if __name__ == "__main__":
    main()
