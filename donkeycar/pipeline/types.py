import os
from typing import Any, List, Optional, cast

from donkeycar.parts.tub_v2 import Tub
from donkeycar.utils import load_image_arr, normalize_image
from typing_extensions import TypedDict

TubRecordDict = TypedDict(
    'TubRecordDict',
    {
        'cam/image_array': str,
        'user/angle': float,
        'user/throttle': float,
        'user/mode': str,
        'imu/accel': Optional[List[float]],
        'imu/gyro': Optional[List[float]],
    }
)


class TubRecord(object):
    def __init__(self, config: Any, base_path: str, underlying: TubRecordDict) -> None:
        self.config = config
        self.base_path = base_path
        self.underlying = underlying
        self._image: Optional[Any] = None

    def image(self, cached=True, normalize=False) -> Any:

        def _get_image(cached=True):
            if self._image is None:
                image_path = self.underlying['cam/image_array']
                full_path = os.path.join(self.base_path, 'images', image_path)
                _image = load_image_arr(full_path, cfg=self.config)
                if cached:
                    self._image = _image
                return _image
            else:
                return self._image

        img_arr = _get_image(cached=cached)
        if normalize:
            img_arr = normalize_image(img_arr)
        return img_arr