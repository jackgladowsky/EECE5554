import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def load_images(path):
    """
    Load images from file paths and convert to RGB.
    """
    images = [cv2.imread(os.path.join(path, file)) for file in sorted(os.listdir(path))]
    images = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in images]

    print(f"Images loaded successfully!")
    
    fig, axs = plt.subplots(1, len(images), figsize=(24, 8))
    for i in range(len(images)):
        axs[i].imshow(images[i])  # Convert BGR to RGB
        axs[i].set_title(f'Image {i+1}')
        axs[i].axis('off')  # Turn off axis labels
    return images

def match_features(des1, des2):
    """
    Match features between two sets of descriptors using BFMatcher with L2 norm and knnMatch.
    """
    bf = cv2.BFMatcher(cv2.NORM_L2)
    matches = bf.knnMatch(des1, des2, k=2)
    
    # Apply ratio test to filter good matches
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)
    
    good_matches = sorted(good_matches, key=lambda x: x.distance)
    return good_matches

def draw_matches(img1, img2, kp1, kp2, matches, num_matches=100):
    """
    Draw the top N feature matches between two images.
    """
    img_matches = cv2.drawMatches(img1, kp1, img2, kp2, matches[:num_matches], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.figure(figsize=(12, 6))
    plt.imshow(img_matches)
    plt.title("Top Feature Matches")
    plt.show()

def warpPerspectivePadded(
        src, dst, H,
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0):
    """Performs a perspective warp with padding.

    Parameters
    ----------
    src : array_like
        source image, to be warped.
    dst : array_like
        destination image, to be padded.
    H : array_like
        `3x3` perspective transformation matrix.

    Returns
    -------
    src_warped : ndarray
        padded and warped source image
    dst_padded : ndarray
        padded destination image, same size as src_warped

    Optional Parameters
    -------------------
    flags : int, optional
        combination of interpolation methods (`cv2.INTER_LINEAR` or
        `cv2.INTER_NEAREST`) and the optional flag `cv2.WARP_INVERSE_MAP`,
        that sets `H` as the inverse transformation (`dst` --> `src`).
    borderMode : int, optional
        pixel extrapolation method (`cv2.BORDER_CONSTANT` or
        `cv2.BORDER_REPLICATE`).
    borderValue : numeric, optional
        value used in case of a constant border; by default, it equals 0.

    See Also
    --------
    warpAffinePadded() : for `2x3` affine transformations
    cv2.warpPerspective(), cv2.warpAffine() : original OpenCV functions
    """

    assert H.shape == (3, 3), \
        'Perspective transformation shape should be (3, 3).\n' \
        + 'Use warpAffinePadded() for (2, 3) affine transformations.'

    H = H / H[2, 2]  # ensure a legal homography
    if flags in (cv2.WARP_INVERSE_MAP,
                 cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
                 cv2.INTER_NEAREST + cv2.WARP_INVERSE_MAP):
        H = cv2.invert(H)[1]
        flags -= cv2.WARP_INVERSE_MAP

    # it is enough to find where the corners of the image go to find
    # the padding bounds; points in clockwise order from origin
    src_h, src_w = src.shape[:2]
    lin_homg_pts = np.array([
        [0, src_w, src_w, 0],
        [0, 0, src_h, src_h],
        [1, 1, 1, 1]])

    # Transform points
    transf_lin_homg_pts = H.dot(lin_homg_pts)
    transf_lin_homg_pts /= transf_lin_homg_pts[2, :]

    # Find min and max points
    min_x = np.floor(np.min(transf_lin_homg_pts[0])).astype(int)
    min_y = np.floor(np.min(transf_lin_homg_pts[1])).astype(int)
    max_x = np.ceil(np.max(transf_lin_homg_pts[0])).astype(int)
    max_y = np.ceil(np.max(transf_lin_homg_pts[1])).astype(int)

    # Add translation to the transformation matrix to shift to positive values
    anchor_x, anchor_y = 0, 0
    transl_transf = np.eye(3, 3)
    if min_x < 0:
        anchor_x = -min_x
        transl_transf[0, 2] += anchor_x
    if min_y < 0:
        anchor_y = -min_y
        transl_transf[1, 2] += anchor_y
    shifted_transf = transl_transf.dot(H)
    shifted_transf /= shifted_transf[2, 2]

    # Create padded destination image
    dst_h, dst_w = dst.shape[:2]

    pad_widths = [anchor_y, max(max_y, dst_h) - dst_h,
                  anchor_x, max(max_x, dst_w) - dst_w]

    dst_padded = cv2.copyMakeBorder(dst, *pad_widths,
                                    borderType=borderMode, value=borderValue)

    dst_pad_h, dst_pad_w = dst_padded.shape[:2]
    src_warped = cv2.warpPerspective(
        src, shifted_transf, (dst_pad_w, dst_pad_h),
        flags=flags, borderMode=borderMode, borderValue=borderValue)

    return dst_padded, src_warped

def masking(bot, top, alpha):

    topmask, botmask = cv2.inRange(top, 0, 0), cv2.inRange(bot, 0, 0)

    # creates final mask of overlaying black pixels

    mask_bot, mask_top = (botmask != 255) & (topmask == 255), (topmask != 255) & (botmask == 255)

    blend = cv2.addWeighted(top, alpha, bot, 1-alpha, 0.0)

    blend[mask_bot] = bot[mask_bot]

    blend[mask_top] = top[mask_top]

    return blend