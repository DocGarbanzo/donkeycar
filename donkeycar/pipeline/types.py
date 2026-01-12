from copy import copy
import os
from enum import Enum
from typing import Any, List, Optional, Iterator, Union, Iterable
import logging
import numpy as np
from donkeycar.config import Config
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.parts.tub_v2 import Tub
from donkeycar.utils import load_image, load_pil_image, binary_to_img, \
    img_to_arr, img_to_binary, arr_to_binary
from typing_extensions import TypedDict
from donkeycar.course_analysis import get_or_compute_segment_id


logger = logging.getLogger(__name__)


class CachePolicy(Enum):
    NOCACHE = 0
    BINARY = 1
    ARRAY = 2


class PctMode(Enum):
    """
    Performance ranking mode for training.

    NONE: No performance ranking (no lap_pct field)
    LAP: Lap-based performance ranking
    SEGMENT: Segment-based performance ranking
    """
    NONE = 0
    LAP = 1
    SEGMENT = 2


TubRecordDict = TypedDict(
    'TubRecordDict',
    {
        '_index': int,
        '_session_id': str,
        'cam/image_array': str,
        'user/angle': float,
        'user/throttle': float,
        'user/mode': str,
        'imu/acl_x': Optional[float],
        'imu/acl_y': Optional[float],
        'imu/acl_z': Optional[float],
        'imu/gyr_x': Optional[float],
        'imu/gyr_y': Optional[float],
        'imu/gyr_z': Optional[float],
        'behavior/one_hot_state_array': Optional[List[float]],
        'localizer/location': Optional[int],
        'car/accel': Optional[List[float]],
        'car/gyro': Optional[List[float]],
        'car/speed': Optional[float],
        'car/lap': Optional[int],
        'lap_pct': Optional[float]
    }
)


# def softmax_plus(n):
#     logn = max(math.log(n), 1)
#     return [(1 / (i+1)) * logn for i in range(n)]


def softmax_plus(n):
    x = np.arange(n) * 20 / n
    return n * np.exp(-x) / np.exp(-x).sum()


class TubRecord(object):
    def __init__(self, config: Config, base_path: str,
                 underlying: TubRecordDict) -> None:
        self.config = config
        self.base_path = base_path
        self.underlying = underlying
        self._cache_policy = CachePolicy[
            getattr(self.config, 'CACHE_POLICY', 'ARRAY')]
        self._cache_images = getattr(self.config, 'CACHE_IMAGES', True)
        self._image: Optional[Any] = None

    def __copy__(self):
        """ Make shallow copies of config and image and full copies of the rest.
        :return TubRecord:    TubRecord copy
        """
        tubrec = TubRecord(self.config,
                           copy(self.base_path),
                           copy(self.underlying))
        tubrec._cache_policy = copy(self._cache_policy)
        tubrec._cache_images = copy(self._cache_images)
        tubrec._image = self._image
        return tubrec

    def image(self, processor=None, as_nparray=True) -> np.ndarray:
        """
        Loads the image.

        :param processor:   Image processing like augmentations or cropping, if
                            not None. Defaults to None.
        :param as_nparray:  Whether to convert the image to a np array of uint8.
                            Defaults to True. If false, returns result of
                            Image.open()
        :return:            Image
        """
        if self._image is None:
            _image = self._extract_image(as_nparray, processor)
        else:
            _image = self._image_from_cache(as_nparray)
            if processor:
                _image = processor(_image)
        return _image

    def _image_from_cache(self, as_nparray):
        """
        Cache policy only supports numpy array format
        :return: Numpy array from cache
        """
        if not as_nparray:
            return self._image

        if self._cache_policy == CachePolicy.NOCACHE:
            raise RuntimeError("Found cached image with policy NOCACHE")
        elif self._cache_policy == CachePolicy.ARRAY:
            return self._image
        elif self._cache_policy == CachePolicy.BINARY:
            return img_to_arr(binary_to_img(self._image))
        else:
            raise RuntimeError(f"Unhandled cache policy {self._cache_policy}")

    def _load_image_and_cache(self, img_path):
        # if no caching, just load but don't cache
        if self._cache_policy == CachePolicy.NOCACHE:
            _image = load_image(img_path, cfg=self.config)
        # if caching full array, load and cache array
        elif self._cache_policy == CachePolicy.ARRAY:
            _image = load_image(img_path, cfg=self.config)
            self._image = _image
        # if caching is binary, only cache binary but return full array
        elif self._cache_policy == CachePolicy.BINARY:
            with open(img_path, 'rb') as f:
                _image = f.read()
                self._image = _image
                _image = img_to_arr(binary_to_img(_image))
        return _image

    def _load_pil_image_and_cache(self, img_path):
        _image = load_pil_image(img_path, cfg=self.config)
        if self._cache_policy != CachePolicy.NOCACHE:
            self._image = _image
        return _image

    def _cache_processed_image(self, image, as_nparray):
        if not as_nparray:
            if self._cache_policy != CachePolicy.NOCACHE:
                self._image = image
            return
        # if numpy and array caching, cache the processed image
        if self._cache_policy == CachePolicy.ARRAY:
            self._image = image
        # if numpy and binary caching, cache binary image, but return
        # numpy
        elif self._cache_policy == CachePolicy.BINARY:
            self._image = arr_to_binary(image)
        # in the case of no caching, nothing needs to be done here

    def _extract_image(self, as_nparray, processor):
        image_path = self.underlying['cam/image_array']
        full_path = os.path.join(self.base_path, 'images', image_path)
        if as_nparray:
            _image = self._load_image_and_cache(full_path)
        else:
            _image = self._load_pil_image_and_cache(full_path)
        if processor:
            # _image is now either numpy or PIL, so processing applies always
            _image = processor(_image)
            self._cache_processed_image(_image, as_nparray)
        return _image

    def extend(self, session_lap_rank, ranking_keys=None,
               pct_mode=PctMode.NONE, segment_id=None):
        """
        Extend record with lap or segment performance rankings.

        :param session_lap_rank: Dictionary of session -> lap -> ranking data
                                For LAP mode: {session_id: {lap: rankings}}
                                For SEGMENT mode: {session_id: {lap: {segment:
                                rankings}}}
        :param ranking_keys: Optional list of keys to extract for lap_pct.
                           If None, uses default ('time', 'distance',
                           'gyro_z_agg')
                           for backward compatibility.
        :param pct_mode: Performance mode (NONE, LAP, or SEGMENT)
        :param segment_id: Pre-computed segment ID (for SEGMENT mode).
                          If provided, uses instead of reading record.
        :return: True if extension succeeded, False otherwise
        """
        if not session_lap_rank:
            return True
        session_id = self.underlying['_session_id']
        lap_i = self.underlying.get('car/lap', 0)

        # Use default keys for backward compatibility
        if ranking_keys is None:
            ranking_keys = ('time', 'distance', 'gyro_z_agg')

        if pct_mode == PctMode.SEGMENT:
            # Use passed segment_id if provided, otherwise read from record
            if segment_id is None:
                segment_id = self.underlying.get('car/segment')
            if segment_id is None:
                return False  # No segment assignment, exclude from training

            lap_dict = session_lap_rank.get(session_id, {}).get(lap_i)
            if lap_dict and segment_id in lap_dict:
                segment_rank_dict = lap_dict[segment_id]
                # Convert dict to list in same order as LAP mode
                lap_pct = [segment_rank_dict[key] for key in ranking_keys
                          if key in segment_rank_dict]
                if lap_pct:
                    self.underlying['lap_pct'] = lap_pct
                    return True  # Successfully populated lap_pct

            return False  # Couldn't populate lap_pct, exclude from training
        else:
            # LAP mode or backward compatibility
            lap_i_dict = session_lap_rank.get(session_id, {}).get(lap_i)
            if lap_i_dict:
                # Extract only keys that exist in lap_i_dict
                lap_pct = [lap_i_dict[key] for key in ranking_keys
                          if key in lap_i_dict]
                if lap_pct:  # Only set if we have values
                    self.underlying['lap_pct'] = lap_pct
                    return True  # Successfully populated lap_pct

            return False  # Couldn't populate lap_pct, exclude from training

    def __repr__(self) -> str:
        return repr(self.underlying)


class TubDataset(object):
    """
    Loads the dataset and creates a TubRecord list (or list of lists).
    """

    def __init__(self, config: Config, tub_paths: List[str],
                 seq_size: int = 0, add_lap_pct: bool = False,
                 ranking_keys: Optional[List[str]] = None,
                 pct_mode: PctMode = PctMode.NONE) -> None:
        """
        Initialize TubDataset.

        :param config: Configuration object
        :param tub_paths: List of paths to tub directories
        :param seq_size: Sequence size for RNN (0 for non-sequential)
        :param add_lap_pct: Whether to add lap_pct to records
        :param ranking_keys: Keys for lap_pct rankings.
                           Defaults: ('time', 'distance', 'gyro_z_agg')
        :param pct_mode: Performance mode (NONE, LAP, or SEGMENT)
        """
        self.config = config
        self.tub_paths = tub_paths
        self.tubs: List[Tub] = [Tub(tub_path, read_only=True)
                                for tub_path in self.tub_paths]
        self.records: List[TubRecord] = list()
        self.train_filter = getattr(config, 'TRAIN_FILTER', None)
        self.compress = getattr(config, 'COMPRESS_SESSIONS_FOR_LAP_STATS', True)
        self.num_bins = getattr(config, 'NUM_BINS_FOR_LAP_STATS', None)
        self.add_lap_pct = add_lap_pct
        self.seq_size = seq_size
        self.ranking_keys = ranking_keys
        self.pct_mode = pct_mode
        logger.info(
            f'TubDataset: lap_pct={self.add_lap_pct} '
            f'compress={self.compress} bins={self.num_bins} '
            f'keys={self.ranking_keys} mode={self.pct_mode}')

    def get_records(self) -> Union[List[TubRecord], List[List[TubRecord]]]:
        """
        Load records from tubs with optional lap performance ranking.

        This method now supports configurable ranking keys and maintains
        better separation of concerns.

        :return: List of TubRecords or list of lists for sequences
        """
        if not self.records:
            filtered_records = 0
            non_ext_records = 0
            used_records = 0
            logger.info(f'Loading tubs from paths {self.tub_paths}')
            session_lap_rank = None

            for tub in self.tubs:
                # Calculate lap or segment performance if needed
                if self.add_lap_pct or self.pct_mode != PctMode.NONE:
                    session_lap_rank = self._calculate_performance_statistics(
                        tub)

                # For SEGMENT mode, lazy-load assigners per session
                assigners = {}
                prev_segments = {}

                # Load and filter records
                for underlying in tub:
                    record = TubRecord(self.config, tub.base_path, underlying)

                    # Apply training filter if configured
                    if self.train_filter and not self.train_filter(record):
                        filtered_records += 1
                        continue

                    # Compute segment_id for SEGMENT mode
                    segment_id = None
                    if self.pct_mode == PctMode.SEGMENT:
                        segment_id = self._get_segment_id(
                            underlying, tub, assigners, prev_segments)

                    # Extend record with rankings (lap or segment)
                    if record.extend(session_lap_rank, self.ranking_keys,
                                   self.pct_mode, segment_id=segment_id):
                        self.records.append(record)
                        used_records += 1
                    else:
                        non_ext_records += 1

            total_records = used_records + filtered_records + non_ext_records
            logger.info(f'Records: # Total {total_records}  # Used '
                        f'{used_records}  # Filtered {filtered_records}  # '
                        f'NonExtended {non_ext_records}')

            # Create sequences if needed
            if self.seq_size > 0:
                seq = Collator(self.seq_size, self.records)
                self.records = list(seq)

        return self.records

    def _calculate_performance_statistics(self, tub: Tub) -> dict:
        """
        Calculate lap or segment performance statistics for a tub.

        Separated from get_records() for better modularity.

        :param tub: Tub to calculate statistics for
        :return: Session performance rank dictionary
                 For LAP mode: {session_id: {lap: [rankings]}}
                 For SEGMENT mode: {session_id: {lap: {segment: [rankings]}}}
        """
        tub_stat = TubStatistics(tub, config=self.config)

        # Determine mode: prefer pct_mode, fall back to add_lap_pct
        if self.pct_mode == PctMode.SEGMENT:
            session_rank = tub_stat.calculate_segment_performance(
                self.config.USE_LAP_0,
                num_bins=self.num_bins)
        elif self.pct_mode == PctMode.LAP or self.add_lap_pct:
            session_rank = tub_stat.calculate_lap_performance(
                self.config.USE_LAP_0,
                num_bins=self.num_bins,
                compress=self.compress)
        else:
            session_rank = None

        return session_rank

    @staticmethod
    def convert_to_weight(session_lap_rank):
        for session, lap_rank in session_lap_rank.items():
            # lap_rank is ordered dictionary of lap number vs lap pct,
            # we replace the value lap pct with a weight given by the order
            num_laps = len(lap_rank)
            weights = softmax_plus(num_laps)
            for i_weight, key in enumerate(lap_rank.keys()):
                lap_rank[key] = weights[i_weight]

    def _get_segment_id(self, underlying, tub, assigners, prev_segments):
        """Get segment ID for record, computing on-the-fly if needed."""
        # First try to get from record (backward compatibility)
        segment_id = underlying.get('car/segment')
        if segment_id is not None:
            return segment_id

        # Compute on-the-fly from metadata
        session_id = underlying['_session_id']
        pos = underlying.get('car/pos')
        return get_or_compute_segment_id(
            session_id, pos, tub.manifest.metadata,
            assigners, prev_segments)

    def close(self):
        logger.info(f'Closing TubDataset')
        for tub in self.tubs:
            tub.close()


class Collator(Iterable[List[TubRecord]]):
    """ Builds a sequence of continuous records for RNN and similar models. """
    def __init__(self, seq_length: int, records: List[TubRecord]):
        """
        :param seq_length:  length of sequence
        :param records:     input record list
        """
        self.records = records
        self.seq_length = seq_length

    @staticmethod
    def is_continuous(rec_1: TubRecord, rec_2: TubRecord) -> bool:
        """
        Checks if second record is next to first record

        :param rec_1:   first record
        :param rec_2:   second record
        :return:        if first record is followed by second record
        """
        it_is = rec_1.underlying['_index'] == rec_2.underlying['_index'] - 1 \
                and '__empty__' not in rec_1.underlying \
                and '__empty__' not in rec_2.underlying
        return it_is

    def __iter__(self) -> Iterator[List[TubRecord]]:
        """ Iterable interface. Returns a generator as Iterator. """
        it = iter(self.records)
        for this_record in it:
            seq = [this_record]
            seq_it = copy(it)
            for next_record in seq_it:
                if self.is_continuous(this_record, next_record) and \
                        len(seq) < self.seq_length:
                    seq.append(next_record)
                    this_record = next_record
                else:
                    break
            if len(seq) == self.seq_length:
                yield seq


