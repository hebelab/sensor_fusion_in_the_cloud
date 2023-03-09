import numpy as np
from PIL import Image
from matplotlib import pyplot as plt

import numpy as np
import cv2

def lpf(img, ncutoff):
    # Apply 2D FFT to the image
    f = np.fft.fft2(img)

    # Shift the zero frequency component to the center of the spectrum
    fshift = np.fft.fftshift(f)

    # Create a circular mask of the same size as the spectrum
    rows, cols = img.shape
    crow, ccol = rows // 2, cols // 2
    mask = np.zeros((rows, cols), np.uint8)
    cutoff = int(min(crow, ccol)*ncutoff)
    cv2.circle(mask, (ccol, crow), cutoff, 1, -1)

    # Apply the mask to the shifted spectrum
    fshift_filtered = fshift * mask

    # Shift the zero frequency component back to the corner of the spectrum
    f_filtered = np.fft.ifftshift(fshift_filtered)

    # Apply the inverse 2D FFT to the filtered spectrum
    img_filtered = np.fft.ifft2(f_filtered)
    img_filtered = np.real(img_filtered)

    return img_filtered


def pg(input, us_rate, ncutoff, gt_mean):

    pg_mean = 0
    filtered = input

    while pg_mean * 1.1 < gt_mean:
        filtered = lpf(filtered,ncutoff)
        filtered[::us_rate, ::us_rate] = input[::us_rate, ::us_rate]
        pg_mean = filtered.mean()
    
    return filtered


image_file = 'baboon.png'

image = Image.open(image_file).convert('L')
x = np.asarray(image)

m = 4;  # downsampling coefficient
y = x[::m, ::m]  # input LR image

z = np.zeros(x.shape, dtype=int)
z[::m, ::m] = y

filtered = z

gt_mean = x.mean()

filtered = pg(z, m, 0.2, gt_mean)

plt.imshow(filtered)
plt.waitforbuttonpress()