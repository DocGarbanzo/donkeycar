import cv2
import numpy as np
from pathlib import Path
import time

Mx = 128  # Natural mean
C = 0.25  # Base line fraction
Ts = 0.15  # Tunable amplitude


def fast_stretch(image, C, Ts, debug=False):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    (h, s, v) = cv2.split(hsv)
    input = v
    shape = input.shape
    rows = shape[0]
    cols = shape[1]
    size = rows * cols
    if debug:
        start = time.time()
    mean = np.mean(input)
    t = mean / 255
    Sl = t * (1 - Ts) + C
    Sh = t * (1 + Ts) + C

    if debug:
        time_taken = (time.time() - start) * 1000
        print('Preprocessing time %s' % time_taken)
        start = time.time()

    histogram = np.bincount(v.reshape(size, ), minlength=256)
    # Walk histogram
    Xl = 0
    Xh = 255
    targetFl = Sl * size
    targetFh = Sh * size

    count = 0
    while count < targetFl and Xl < 256:
        count += histogram[Xl]
        Xl += 1

    count = 0
    while count < size - targetFh and Xh > -1:
        count += histogram[Xh]
        Xh -= 1

    if debug:
        time_taken = (time.time() - start) * 1000
        print('Histogram Binning %s' % time_taken)
        start = time.time()

    # Vectorized ops
    output = np.where(input <= Xl, 0, input)
    output = np.where(output >= Xh, 255, output)
    # flooring denominator by 1, so image stays in [0, 255] if Xl == Xh
    output = np.where(np.logical_and(output > Xl, output < Xh),
                      255 * (output - Xl) / max((Xh - Xl), 1),
                      output)
    # convert to uint8
    output = np.asarray(output, dtype='uint8')
    output = cv2.merge((h, s, output))
    output = cv2.cvtColor(output, cv2.COLOR_HSV2RGB)

    if debug:
        time_taken = (time.time() - start) * 1000
        print('Vector Ops %s' % time_taken)
        print('t', t, 'Sl', Sl, 'Sh', Sh, 'Xl', Xl, 'Xh', Xh)

    return output


if __name__ == "__main__":
    path = Path('images/Lenna.jpg')
    image = cv2.imread(path.as_posix())
    # Ensure RGB
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_data = np.asarray(rgb, dtype=np.uint8)
    stretched = fast_stretch(image_data, debug=True)
    stretched_bgr = cv2.cvtColor(stretched, cv2.COLOR_RGB2BGR)
    cv2.imshow('Original', image)
    cv2.imshow('Contrast Stretched', stretched)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
