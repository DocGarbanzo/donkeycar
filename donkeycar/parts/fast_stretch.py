import cv2
import numpy as np
from pathlib import Path
import time

Mx = 128  # Natural mean
C = 0.25  # Base line fraction
Ts = 0.15  # Tunable amplitude
Tr = 0.8  # Threshold
T = -0.50  # Gamma boost
Epsilon = 1  # Epsilon


def fast_stretch(images, C, Ts):
    hues = []
    saturations = []
    Xls = []
    Xhs = []
    inputs = []
    outputs = []
    for img in images:
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        (h, s, v) = cv2.split(hsv)
        inputs.append(v)
        hues.append(h)
        saturations.append(s)
        shape = v.shape
        rows = shape[0]
        cols = shape[1]
        size = rows * cols

        mean = np.mean(v)
        t = mean / 255
        Sl = t * (1 - Ts) + C
        Sh = t * (1 + Ts) + C

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

        Xls.append(Xl)
        Xhs.append(Xh)

    inputs_arr = np.array(inputs)
    Xl_arr = np.array(Xls)
    Xh_arr = np.array(Xhs)
    # Vectorized ops
    output = np.where(inputs_arr <= Xl_arr, 0, inputs_arr)
    output = np.where(output >= Xh_arr, 255, output)
    output = np.where(np.logical_and(output > Xl_arr, output < Xh_arr),
                      255 * (output - Xl_arr) / max((Xh_arr - Xl_arr), Epsilon),
                      output)
    # max to 255 and integer casting
    output = np.where(output > 255., 255., output)
    output = np.asarray(output, dtype='uint8')
    for hue, sat, out_single in zip(hues, saturations, output):
        out_img = cv2.merge((hue, sat, out_single))
        out_img = cv2.cvtColor(out_img, cv2.COLOR_HSV2RGB)
        outputs.append(out_img)
    return outputs


if __name__ == "__main__":
    path = Path('images/Lenna.jpg')
    image = cv2.imread(path.as_posix())
    # Ensure BGR
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    image_data = np.asarray(bgr, dtype=np.uint8)

    stretched = fast_stretch(image_data)
    cv2.imshow('Original', image)
    cv2.imshow('Contrast Stretched', stretched)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
