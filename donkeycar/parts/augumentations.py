import cv2
import numpy as np
import imgaug.augmenters as iaa


class Augumentations(object):
    """
    Some ready to use image augumentations.
    """

    @classmethod
    def crop(cls, left, right, top, bottom, keep_size=False):
        """
        The image augumentation sequence.
        Crops based on a region of interest among other things.
        left, right, top & bottom are the number of pixels to crop.
        """

        augmentation = iaa.Sequential([
            iaa.Crop(
                px=(top, right, bottom, left),
                keep_size=keep_size
            ),
        ])
        return augmentation

    @classmethod
    def trapezoidal_mask(cls, lower_left, lower_right, upper_left, upper_right, min_y, max_y):
        """
        Uses a binary mask to generate a trapezoidal region of interest.
        Especially useful in filtering out uninteresting features from an
        input image.
        """

        def _transform_images(images, random_state, parents, hooks):
            # Transform a batch of images
            transformed = []
            mask = None
            for image in images:
                if mask is None:
                    mask = np.zeros(image.shape, dtype='int32')
                    # # # # # # # # # # # # #
                    #       ul     ur          min_y
                    #
                    #
                    #
                    #    ll             lr     max_y
                    points = [
                        [upper_left, min_y],
                        [upper_right, min_y],
                        [lower_right, max_y],
                        [lower_left, max_y]
                    ]
                    cv2.fillConvexPoly(mask, np.array(points, dtype=np.int32), [255, 255, 255])
                    mask = np.asarray(mask, dtype='bool')

                masked = np.multiply(image, mask)
                transformed.append(masked)

            return transformed

        def _transform_keypoints(keypoints_on_images, random_state, parents, hooks):
            # No-op
            return keypoints_on_images

        augmentation = iaa.Sequential([
            iaa.Lambda(func_images=_transform_images, func_keypoints=_transform_keypoints)
        ])

        return augmentation

    @classmethod
    def brightness(cls, rand_lower=None, rand_upper=None, absolute=0.3125):
        """
        Returns a brightness adjusted image, either applying a random factor
        to the brightness or adjusting the brightness to a given fixed level.

        :param float or None rand_lower:    lower range factor for random
                                            adjustment
        :param float or None rand_upper:    upper range factor for random
                                            adjustment
        :param float absolute:              absolute brightness level
        :return:                            augmenter with above properties
        """

        if rand_lower:
            assert rand_upper and rand_upper > rand_lower, \
                'Need to provide rand_upper > rand_lower'
            aug = iaa.Multiply((rand_lower, rand_upper))

        else:
            assert 0 < absolute < 1.0, "requires image brightness in (0,1)"

            def img_func(images, random_state, parents, hooks):
                transformed = []
                for img in images:
                    img_size = 1
                    for i in img.shape:
                        img_size *= i
                    img_brightness = np.sum(img) / img_size
                    img_transformed = img
                    if img_brightness > 0:
                        img_transformed = img * absolute / img_brightness
                        np.clip(img_transformed, 0, 1, out=img_transformed)
                    transformed.append(img_transformed)

                return transformed

            def keypoint_func(keypoints, random_state, parents, hooks):
                # no op
                return keypoints

            aug = iaa.Lambda(img_func, keypoint_func)

        return aug

