import numpy as np
from scipy import signal
from scipy.ndimage import maximum_filter, generate_binary_structure
import matplotlib.pyplot as plt

def harris(I, N=100, **kwargs):
    """
    Harris corner detector.
    This function implements a version of the Harris corner detector which
    has the ability to calculate the eigenvalues of the gradient matrix
    directly.  This is opposed to calculating the corner response function as
    originally proposed by Harris in:

    C. Harris and M. Stephens.  "A Combined Corner and Edge
    Detector", Proceedings of the 4th Alvey Vision Conference,
    Manchester, U.K. pgs 147-151, 1988
    
    INPUT:
        I: grayscale image
    PARAMETERS:
        N: maximum number of interest points to return
        disp: whether to display results
        thresh: threshold value for smallest acceptable value of response function
        hsize: size of the smoothing Gaussian mask
        sigma: standard deviation of the Gaussian filter
        tile: list [y, x], break the image into regions to distribute feature points more uniformly
        mask: array of ones defining where to compute feature points
        eig: use smallest eigenvalue as response function
        fft: perform smoothing filtering in frequency domain
    OUTPUT:
        y, x: row/column locations of interest points
        m: corner response function value associated with that point
    """
    param = {
        'disp': False,
        'N': N,
        'thresh': 0,
        'hsize': 3,
        'sigma': 0.5,
        'eig': False,
        'tile': [1,1],
        'mask': None,
        'fft': False,
    }
    param.update(kwargs)

    I = I.astype(np.float64)

    nr, nc = I.shape

    # Create gradient masks
    dx = np.array([[-1, 0, 1],
                   [-1, 0, 1],
                   [-1, 0, 1]], dtype=np.float64) / 3

    dy = dx.T

    # Calculate image gradients
    Ix = signal.convolve2d(I, dx, boundary='symm', mode='same')
    Iy = signal.convolve2d(I, dy, boundary='symm', mode='same')

    # Calculate gradient products
    IxIx = Ix * Ix
    IyIy = Iy * Iy
    IxIy = Ix * Iy

    # Smooth squared image gradients
    hsize = param['hsize']
    sigma = param['sigma']

    gmask = gaussian_kernel(hsize, sigma)

    if not param['fft']:
        IxIx = signal.convolve2d(IxIx, gmask, boundary='symm', mode='same')
        IyIy = signal.convolve2d(IyIy, gmask, boundary='symm', mode='same')
        IxIy = signal.convolve2d(IxIy, gmask, boundary='symm', mode='same')
    else:
        # Perform convolution in frequency domain
        m = IxIx.shape[0] + gmask.shape[0] - 1
        n = IxIx.shape[1] + gmask.shape[1] - 1
        G = np.fft.fft2(gmask, s=(m,n))
        IxIx = np.real(np.fft.ifft2(np.fft.fft2(IxIx, s=(m,n)) * G))
        IyIy = np.real(np.fft.ifft2(np.fft.fft2(IyIy, s=(m,n)) * G))
        IxIy = np.real(np.fft.ifft2(np.fft.fft2(IxIy, s=(m,n)) * G))
        # Keep 'same' portion
        w = (hsize - 1) // 2  # hsize is assumed to be odd
        IxIx = IxIx[w:w+nr, w:w+nc]
        IyIy = IyIy[w:w+nr, w:w+nc]
        IxIy = IxIy[w:w+nr, w:w+nc]

    # Calculate the eigenvalues
    B = IxIx + IyIy
    sqrt_term = np.sqrt(B**2 - 4*(IxIx * IyIy - IxIy**2))
    lambda1 = (B + sqrt_term) / 2
    lambda2 = (B - sqrt_term) / 2

    # Corner response function
    if param['eig']:
        # Minimum eigenvalue
        R = np.minimum(lambda1, lambda2)
    else:
        # Harris corner response function
        R = lambda1 * lambda2 - 0.04 * (lambda1 + lambda2)**2

    # Apply mask if provided
    if param['mask'] is not None:
        mask = param['mask']
        if mask.shape != R.shape:
            raise ValueError("Mask must be the same size as the image.")
    else:
        mask = np.ones_like(R, dtype=bool)

    # Find local maxima
    neighborhood = generate_binary_structure(2, 2)
    local_max = (R == maximum_filter(R, footprint=neighborhood))
    Maxima = (local_max & mask)
    Maxima_R = Maxima * R

    # Get indices where Maxima > thresh
    indices = np.argwhere(Maxima_R > param['thresh'])
    m_values = Maxima_R[indices[:,0], indices[:,1]]

    # Sort interest points by response function
    sorted_indices = np.argsort(m_values)
    # Flip so largest response points are first
    sorted_indices = sorted_indices[::-1]

    i = indices[sorted_indices,0]
    j = indices[sorted_indices,1]
    m = m_values[sorted_indices]

    # Process image regionally
    tile = param['tile']
    if tile[0] > 1 and tile[1] > 1:
        ii = []
        jj = []
        mm = []
        Npts_per_region = int(round(param['N'] / (tile[0]*tile[1])))
        xx = np.round(np.linspace(0, nc, tile[1]+1)).astype(int)
        yy = np.round(np.linspace(0, nr, tile[0]+1)).astype(int)
        for pp in range(1, len(xx)):
            idx = np.where((j >= xx[pp-1]) & (j < xx[pp]))[0]
            for qq in range(1, len(yy)):
                idy = np.where((i >= yy[qq-1]) & (i < yy[qq]))[0]
                ind = np.intersect1d(idx, idy)
                ind = ind[:min(len(ind), Npts_per_region)]
                ii.extend(i[ind])
                jj.extend(j[ind])
                mm.extend(m[ind])
        ii = np.array(ii)
        jj = np.array(jj)
        mm = np.array(mm)
    else:
        ii = i[:param['N']]
        jj = j[:param['N']]
        mm = m[:param['N']]

    if param['disp']:
        # Overlay corner points on original image
        plt.figure()
        plt.imshow(I, cmap='gray')
        plt.plot(jj, ii, 'y+')
        plt.show()

    return ii, jj, mm

def gaussian_kernel(size, sigma):
    """Generate a 2D Gaussian kernel."""
    size = int(size)
    x, y = np.mgrid[-(size//2):(size//2)+1, -(size//2):(size//2)+1]
    g = np.exp(-(x**2 + y**2)/(2*sigma**2))
    return g / g.sum()
