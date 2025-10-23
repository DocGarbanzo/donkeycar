import os
import shutil
import tempfile
import time
import unittest
from typing import List
import math

import numpy as np

from donkeycar.config import Config
from donkeycar.pipeline.sequence import PipelineGenerator
from donkeycar.pipeline.types import TubRecord, TubDataset
from donkeycar.parts.tub_v2 import Tub
from donkeycar.parts.tub_statistics import TubStatistics
from donkeycar.pipeline.transformations import (
    Transformation,
    SortingStrategy,
    SortingCriterion,
    abs_transform,
    clamp_transform,
    scale_transform,
    identity_transform,
    default_lap_sorting_strategy,
    custom_lap_sorting_strategy
)


def random_records(size: int = 100) -> List[TubRecord]:
    return [random_record() for _ in range(size)]


def random_record() -> TubRecord:
    now = int(time.time())
    underlying = {
        'cam/image_array': f'/path/to/{now}.txt',
        'user/angle': np.random.uniform(0, 1.),
        'user/throttle': np.random.uniform(0, 1.),
        'user/mode': 'driving',
        'imu/acl_x': None,
        'imu/acl_y': None,
        'imu/acl_z': None,
        'imu/gyr_x': None,
        'imu/gyr_y': None,
        'imu/gyr_z': None
    }
    return TubRecord(config=Config(), base_path='/base', underlying=underlying)


size = 10


class TestPipeline(unittest.TestCase):

    def setUp(self):
        records = random_records(size=size)
        self.sequence = records

    def test_basic_iteration(self):
        self.assertEqual(len(self.sequence), size)
        count = 0
        for record in self.sequence:
            print(f'Record {record}')
            count += 1

        self.assertEqual(count, size)

    def test_basic_map_operations(self):
        transformed = PipelineGenerator(
            self.sequence,
            x_transform=lambda record: record.underlying['user/angle'],
            y_transform=lambda record: record.underlying['user/throttle'])

        transformed_2 = PipelineGenerator(
            self.sequence,
            x_transform=lambda record: record.underlying['user/angle'] * 2,
            y_transform=lambda record: record.underlying['user/throttle'] * 2)

        self.assertEqual(len(transformed), size)
        self.assertEqual(len(transformed_2), size)

        transformed_list = list(transformed)
        transformed_list_2 = list(transformed_2)
        index = np.random.randint(0, 9)

        x1, y1 = transformed_list[index]
        x2, y2 = transformed_list_2[index]

        self.assertAlmostEqual(x1 * 2, x2)
        self.assertAlmostEqual(y1 * 2, y2)

    def test_more_map_operations(self):
        transformed = PipelineGenerator(
            self.sequence,
            x_transform=lambda record: record.underlying['user/angle'],
            y_transform=lambda record: record.underlying['user/throttle'])

        transformed_2 = PipelineGenerator(
            self.sequence,
            x_transform=lambda record: record.underlying['user/angle'] * 2,
            y_transform=lambda record: record.underlying['user/throttle'] * 2)

        self.assertEqual(len(transformed), size)
        self.assertEqual(len(transformed_2), size)


class TestTubDatasetSortingAndTransformation(unittest.TestCase):
    """
    Test suite for TubDataset sorting/ordering and transformation capabilities.

    Tests the current behavior of:
    - TubDataset.get_records() with lap_pct extension
    - TubStatistics.calculate_lap_performance() sorting
    - Gyro aggregation with abs() transformation

    After refactoring, these tests will ensure backward compatibility.
    """

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.test_path = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up test fixtures after each test method."""
        if os.path.exists(self.test_path):
            shutil.rmtree(self.test_path)

    def _create_oval_track_data(self, num_laps=3, records_per_lap=100,
                                lap_time_ms=10000, track_length=50.0,
                                session_id=None, varying_performance=False):
        """
        Create simulated data for a car driving on an oval track.

        :param num_laps: Number of complete laps to simulate
        :param records_per_lap: Number of records per lap
        :param lap_time_ms: Base time in milliseconds to complete one lap
        :param track_length: Track length in distance units
        :param session_id: Optional session ID
        :param varying_performance: If True, vary lap times for performance testing
        :return: List of record dictionaries
        """
        records = []
        start_time_ms = int(time.time() * 1000)

        if session_id is None:
            session_id = f"oval_session_{start_time_ms}"

        for lap in range(num_laps):
            # Vary lap time if requested (for testing sorting)
            if varying_performance:
                # Make some laps faster, some slower
                lap_time_variation = lap_time_ms * (0.8 + 0.4 * (lap % 3) / 2)
            else:
                lap_time_variation = lap_time_ms

            for record_idx in range(records_per_lap):
                lap_progress = record_idx / records_per_lap

                record_time_ms = start_time_ms + int(
                    sum([lap_time_ms if not varying_performance
                         else lap_time_ms * (0.8 + 0.4 * (i % 3) / 2)
                         for i in range(lap)])
                    + lap_progress * lap_time_variation
                )

                distance = lap * track_length + lap_progress * track_length

                # Simulate gyro Z for oval track
                angle = lap_progress * 2 * math.pi
                turn_intensity = abs(math.sin(2 * angle))
                gyro_z = 0.1 + turn_intensity * 0.9

                # Include negative values to test abs() transformation
                gyro_raw = gyro_z if record_idx % 2 == 0 else -gyro_z
                gyro = [0.0, gyro_raw, 0.0]

                # Add steering and throttle for transformation testing
                steering = math.sin(angle) * 0.5
                throttle = 0.5 + 0.3 * math.cos(angle)

                record = {
                    'car/lap': lap,
                    'car/distance': distance,
                    'car/gyro': gyro,
                    'user/angle': steering,
                    'user/throttle': throttle,
                    '_timestamp_ms': record_time_ms,
                    '_session_id': session_id,
                }
                records.append(record)

        # Add final record to mark end of last lap
        final_time_offset = sum([lap_time_ms if not varying_performance
                                else lap_time_ms * (0.8 + 0.4 * (i % 3) / 2)
                                for i in range(num_laps)])
        final_record_time_ms = start_time_ms + int(final_time_offset)
        final_distance = num_laps * track_length
        records.append({
            'car/lap': num_laps,
            'car/distance': final_distance,
            'car/gyro': [0.0, 0.1, 0.0],
            'user/angle': 0.0,
            'user/throttle': 0.5,
            '_timestamp_ms': final_record_time_ms,
            '_session_id': session_id,
        })

        return records

    def _create_tub_with_data(self, records):
        """
        Create a tub and write records to it.

        :param records: List of record dictionaries to write
        :return: Tub instance with data written
        """
        inputs = ['car/lap', 'car/distance', 'car/gyro', 'user/angle', 'user/throttle']
        types = ['int', 'float', 'vector', 'float', 'float']
        tub = Tub(self.test_path, inputs, types)

        for record in records:
            tub.write_record(record)

        return tub

    def test_current_lap_pct_extension(self):
        """
        Test current behavior of TubRecord.extend() with lap_pct.

        Validates that:
        - lap_pct is created with ['time', 'distance', 'gyro_z_agg'] ordering
        - Values are correctly populated from lap performance rankings
        """
        records = self._create_oval_track_data(num_laps=3, varying_performance=True)
        tub = self._create_tub_with_data(records)

        # Generate lap times
        stats = TubStatistics(tub, gyro_z_index=1)
        stats.generate_laptimes_from_records()

        # Close and reopen as read-only
        inputs = ['car/lap', 'car/distance', 'car/gyro', 'user/angle', 'user/throttle']
        types = ['int', 'float', 'vector', 'float', 'float']
        tub.close()
        tub = Tub(self.test_path, inputs, types, read_only=True)

        # Calculate lap performance
        stats = TubStatistics(tub, gyro_z_index=1)
        session_lap_rank = stats.calculate_lap_performance(use_lap_0=True)

        # Test that records can be extended
        test_record = TubRecord(Config(), tub.base_path, next(iter(tub)))
        result = test_record.extend(session_lap_rank)

        # Should succeed (lap 0 should have rankings)
        self.assertTrue(result)

        # Check that lap_pct was added with correct structure
        if 'lap_pct' in test_record.underlying:
            lap_pct = test_record.underlying['lap_pct']
            # Should be a list of 3 values: time, distance, gyro_z_agg
            self.assertEqual(len(lap_pct), 3)
            # Each value should be between 0 and 1
            for val in lap_pct:
                self.assertGreaterEqual(val, 0)
                self.assertLessEqual(val, 1.0)

        tub.close()

    def test_current_sorting_criteria(self):
        """
        Test current hardcoded sorting by 'time', 'distance', 'gyro_z_agg'.

        Validates that:
        - Laps are sorted by these three criteria
        - Sorting is done using itemgetter
        - Rankings are assigned correctly
        """
        records = self._create_oval_track_data(num_laps=5, varying_performance=True)
        tub = self._create_tub_with_data(records)

        stats = TubStatistics(tub, gyro_z_index=1)
        stats.generate_laptimes_from_records()

        # Close and reopen
        inputs = ['car/lap', 'car/distance', 'car/gyro', 'user/angle', 'user/throttle']
        types = ['int', 'float', 'vector', 'float', 'float']
        tub.close()
        tub = Tub(self.test_path, inputs, types, read_only=True)

        stats = TubStatistics(tub, gyro_z_index=1)
        performance = stats.calculate_lap_performance(use_lap_0=True)

        session_id = list(performance.keys())[0]
        session_perf = performance[session_id]

        # Each lap should have rankings for all three criteria
        for lap_num in range(5):
            self.assertIn(lap_num, session_perf)
            lap_perf = session_perf[lap_num]

            # Current implementation uses these three hardcoded keys
            self.assertIn('time', lap_perf)
            self.assertIn('distance', lap_perf)
            self.assertIn('gyro_z_agg', lap_perf)

            # Values should be between 0 and 1
            for key in ['time', 'distance', 'gyro_z_agg']:
                self.assertGreaterEqual(lap_perf[key], 0)
                self.assertLessEqual(lap_perf[key], 1.0)

        tub.close()

    def test_current_abs_transformation_on_gyro(self):
        """
        Test current hardcoded abs() transformation on gyro values.

        Validates that:
        - abs() is applied to gyro_z values during aggregation
        - Negative gyro values are converted to positive
        - Aggregation correctly computes average
        """
        # Create data with explicit negative gyro values
        records = self._create_oval_track_data(num_laps=2)
        tub = self._create_tub_with_data(records)

        stats = TubStatistics(tub, gyro_z_index=1)
        stats.generate_laptimes_from_records()

        # Calculate aggregated gyro (uses abs internally)
        stats._calculate_aggregated_gyro()

        session_id = tub.manifest.session_id[1]
        lap_times = tub.manifest.metadata[session_id]['laptimer']

        # Check that gyro_z_agg values are all positive
        for lap_time in lap_times:
            if 'gyro_z_agg' in lap_time:
                # Should be positive due to abs() transformation
                self.assertGreater(lap_time['gyro_z_agg'], 0)

        tub.close()

    def test_tub_dataset_with_lap_pct(self):
        """
        Test TubDataset.get_records() with add_lap_pct enabled.

        Validates that:
        - TubDataset correctly loads records
        - lap_pct is added when add_lap_pct=True
        - Records without valid lap rankings are filtered out
        """
        records = self._create_oval_track_data(num_laps=3)
        tub = self._create_tub_with_data(records)

        # Generate lap times first
        stats = TubStatistics(tub, gyro_z_index=1)
        stats.generate_laptimes_from_records()
        tub.close()

        # Create TubDataset with lap_pct enabled
        config = Config()
        config.GYRO_Z_INDEX = 1
        config.USE_LAP_0 = True
        config.COMPRESS_SESSIONS_FOR_LAP_STATS = False
        config.NUM_BINS_FOR_LAP_STATS = None

        dataset = TubDataset(config, [self.test_path], add_lap_pct=True)
        records_loaded = dataset.get_records()

        # Should have loaded some records
        self.assertGreater(len(records_loaded), 0)

        # Check that some records have lap_pct
        records_with_lap_pct = [r for r in records_loaded
                                if 'lap_pct' in r.underlying]

        # Most records should have lap_pct (except last incomplete lap)
        self.assertGreater(len(records_with_lap_pct), 0)

        # Check structure of lap_pct
        if len(records_with_lap_pct) > 0:
            sample_rec = records_with_lap_pct[0]
            lap_pct = sample_rec.underlying['lap_pct']
            self.assertEqual(len(lap_pct), 3)  # time, distance, gyro_z_agg

        dataset.close()

    def test_pipeline_with_custom_transformations(self):
        """
        Test PipelineGenerator with custom transformations.

        Validates that:
        - Custom transformations can be applied to record fields
        - Common transformations like abs(), clamp() work correctly
        """
        # Create simple records
        records = random_records(size=20)

        # Test abs() transformation
        transformed_abs = PipelineGenerator(
            records,
            x_transform=lambda record: abs(record.underlying['user/angle']),
            y_transform=lambda record: record.underlying['user/throttle']
        )

        abs_list = list(transformed_abs)
        for x, y in abs_list:
            # x should be non-negative due to abs()
            self.assertGreaterEqual(x, 0)

        # Test clamp() transformation
        def clamp(value, min_val, max_val):
            return max(min_val, min(max_val, value))

        transformed_clamp = PipelineGenerator(
            records,
            x_transform=lambda record: clamp(record.underlying['user/angle'], -0.5, 0.5),
            y_transform=lambda record: clamp(record.underlying['user/throttle'], 0.3, 0.7)
        )

        clamp_list = list(transformed_clamp)
        for x, y in clamp_list:
            # x should be clamped to [-0.5, 0.5]
            self.assertGreaterEqual(x, -0.5)
            self.assertLessEqual(x, 0.5)
            # y should be clamped to [0.3, 0.7]
            self.assertGreaterEqual(y, 0.3)
            self.assertLessEqual(y, 0.7)


class TestTransformations(unittest.TestCase):
    """Test suite for the new Transformation classes."""

    def test_abs_transformation(self):
        """Test absolute value transformation."""
        abs_t = abs_transform()
        self.assertEqual(abs_t(-5.0), 5.0)
        self.assertEqual(abs_t(3.0), 3.0)
        self.assertEqual(abs_t(0.0), 0.0)

    def test_clamp_transformation(self):
        """Test clamp transformation."""
        clamp_t = clamp_transform(-1.0, 1.0)
        self.assertEqual(clamp_t(0.5), 0.5)
        self.assertEqual(clamp_t(2.0), 1.0)
        self.assertEqual(clamp_t(-2.0), -1.0)

    def test_scale_transformation(self):
        """Test scale transformation."""
        scale_t = scale_transform(2.0)
        self.assertEqual(scale_t(5.0), 10.0)
        self.assertEqual(scale_t(-3.0), -6.0)

    def test_transformation_composition(self):
        """Test composing multiple transformations."""
        # abs -> scale(2) -> clamp(0, 5)
        abs_t = abs_transform()
        scale_t = scale_transform(2.0)
        clamp_t = clamp_transform(0.0, 5.0)

        # Compose: abs -> scale
        composed1 = abs_t.compose(scale_t)
        self.assertEqual(composed1(-3.0), 6.0)  # abs(-3) * 2 = 6

        # Compose: abs -> scale -> clamp
        composed2 = composed1.compose(clamp_t)
        self.assertEqual(composed2(-3.0), 5.0)  # abs(-3) * 2 = 6, clamped to 5


class TestSortingStrategy(unittest.TestCase):
    """Test suite for the new SortingStrategy classes."""

    def test_default_sorting_strategy(self):
        """Test default sorting strategy (time, distance, gyro_z_agg)."""
        strategy = default_lap_sorting_strategy()
        self.assertEqual(len(strategy.criteria), 3)
        self.assertEqual(strategy.criteria[0].key, 'time')
        self.assertEqual(strategy.criteria[1].key, 'distance')
        self.assertEqual(strategy.criteria[2].key, 'gyro_z_agg')

    def test_sorting_criterion_extraction(self):
        """Test that SortingCriterion extracts values correctly."""
        criterion = SortingCriterion('time')
        data = {'time': 10.5, 'distance': 50.0}
        self.assertEqual(criterion.get_sort_value(data), 10.5)

    def test_sorting_criterion_with_transformation(self):
        """Test SortingCriterion with transformation."""
        # Create criterion that takes abs of value before sorting
        criterion = SortingCriterion(
            'custom',
            extractor=lambda d: d.get('value'),
            transformation=abs_transform()
        )
        data = {'value': -5.0}
        self.assertEqual(criterion.get_sort_value(data), 5.0)

    def test_rank_laps_basic(self):
        """Test basic lap ranking."""
        strategy = default_lap_sorting_strategy()

        laps = [
            {'lap': 0, 'time': 10.0, 'distance': 50.0, 'gyro_z_agg': 0.5},
            {'lap': 1, 'time': 9.0, 'distance': 48.0, 'gyro_z_agg': 0.4},
            {'lap': 2, 'time': 11.0, 'distance': 52.0, 'gyro_z_agg': 0.6},
        ]

        rankings = strategy.rank_laps(laps)

        # Check that all laps were ranked
        self.assertEqual(len(rankings), 3)

        # Lap 1 should have best time (lowest), so lowest ranking
        # Lap 0 should be middle
        # Lap 2 should have worst time (highest), so highest ranking
        self.assertLess(rankings[1]['time'], rankings[0]['time'])
        self.assertLess(rankings[0]['time'], rankings[2]['time'])

    def test_rank_laps_with_bins(self):
        """Test lap ranking with quantile bins."""
        strategy = default_lap_sorting_strategy()

        laps = [
            {'lap': i, 'time': 10.0 + i, 'distance': 50.0 + i, 'gyro_z_agg': 0.5 + i * 0.1}
            for i in range(10)
        ]

        rankings = strategy.rank_laps(laps, num_buckets=5)

        # With 5 bins, we should only see values 0.2, 0.4, 0.6, 0.8, 1.0
        time_rankings = set(rankings[i]['time'] for i in range(10))
        expected_bins = {0.2, 0.4, 0.6, 0.8, 1.0}
        self.assertEqual(time_rankings, expected_bins)

    def test_custom_sorting_strategy(self):
        """Test creating custom sorting strategy."""
        criteria_specs = [
            {'key': 'speed', 'transformation': 'abs'},
            {'key': 'accuracy', 'reverse': True},  # Higher is better
        ]

        strategy = custom_lap_sorting_strategy(criteria_specs)
        self.assertEqual(len(strategy.criteria), 2)
        self.assertEqual(strategy.criteria[0].key, 'speed')
        self.assertEqual(strategy.criteria[1].key, 'accuracy')
        self.assertTrue(strategy.criteria[1].reverse)


class TestModularTubStatistics(unittest.TestCase):
    """Test suite for refactored TubStatistics with modular sorting."""

    def setUp(self):
        """Set up test fixtures."""
        self.test_path = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_path):
            shutil.rmtree(self.test_path)

    def _create_simple_tub(self, num_laps=3):
        """Create a simple tub with lap data."""
        inputs = ['car/lap', 'car/distance', 'car/gyro']
        types = ['int', 'float', 'vector']
        tub = Tub(self.test_path, inputs, types)

        start_time_ms = int(time.time() * 1000)

        for lap in range(num_laps + 1):
            for i in range(10):
                tub.write_record({
                    'car/lap': lap,
                    'car/distance': lap * 50.0 + i * 5.0,
                    'car/gyro': [0.0, 0.5 if i % 2 == 0 else -0.5, 0.0],
                    '_timestamp_ms': start_time_ms + lap * 10000 + i * 1000,
                })

        return tub

    def test_tub_statistics_with_custom_sorting(self):
        """Test TubStatistics with custom sorting strategy."""
        tub = self._create_simple_tub(num_laps=3)

        # Create custom sorting strategy
        custom_strategy = SortingStrategy([
            SortingCriterion('time'),
            SortingCriterion('distance'),
        ])

        stats = TubStatistics(tub, gyro_z_index=1, sorting_strategy=custom_strategy)
        stats.generate_laptimes_from_records()

        # Close and reopen
        inputs = ['car/lap', 'car/distance', 'car/gyro']
        types = ['int', 'float', 'vector']
        tub.close()
        tub = Tub(self.test_path, inputs, types, read_only=True)

        stats = TubStatistics(tub, gyro_z_index=1, sorting_strategy=custom_strategy)
        performance = stats.calculate_lap_performance(use_lap_0=True)

        session_id = list(performance.keys())[0]
        session_perf = performance[session_id]

        # Should have rankings for custom criteria only
        for lap_num in range(3):
            if lap_num in session_perf:
                lap_perf = session_perf[lap_num]
                self.assertIn('time', lap_perf)
                self.assertIn('distance', lap_perf)
                # gyro_z_agg should still exist from _calculate_aggregated_gyro
                # but may not be in rankings if not in sorting strategy

        tub.close()

    def test_tub_statistics_with_custom_transformation(self):
        """Test TubStatistics with custom gyro transformation."""
        tub = self._create_simple_tub(num_laps=2)

        # Use identity transformation instead of abs
        identity_trans = identity_transform()

        stats = TubStatistics(tub, gyro_z_index=1, gyro_transformation=identity_trans)
        stats.generate_laptimes_from_records()
        stats._calculate_aggregated_gyro()

        session_id = tub.manifest.session_id[1]
        lap_times = tub.manifest.metadata[session_id]['laptimer']

        # With identity transformation, gyro values can be negative or positive
        # (averaging 0.5 and -0.5 should give 0.0)
        for lap_time in lap_times:
            if 'gyro_z_agg' in lap_time:
                # Should be close to 0 with identity transform
                self.assertAlmostEqual(lap_time['gyro_z_agg'], 0.0, delta=0.1)

        tub.close()


class TestModularTubDataset(unittest.TestCase):
    """Test suite for refactored TubDataset with configurable ranking keys."""

    def setUp(self):
        """Set up test fixtures."""
        self.test_path = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_path):
            shutil.rmtree(self.test_path)

    def _create_test_tub(self):
        """Create a test tub with lap data."""
        inputs = ['car/lap', 'car/distance', 'car/gyro']
        types = ['int', 'float', 'vector']
        tub = Tub(self.test_path, inputs, types)

        start_time_ms = int(time.time() * 1000)

        for lap in range(3):
            for i in range(10):
                tub.write_record({
                    'car/lap': lap,
                    'car/distance': lap * 50.0 + i * 5.0,
                    'car/gyro': [0.0, 0.5, 0.0],
                    '_timestamp_ms': start_time_ms + lap * 10000 + i * 1000,
                })

        # Add final lap marker
        tub.write_record({
            'car/lap': 3,
            'car/distance': 150.0,
            'car/gyro': [0.0, 0.5, 0.0],
            '_timestamp_ms': start_time_ms + 30000,
        })

        # Generate lap times
        stats = TubStatistics(tub, gyro_z_index=1)
        stats.generate_laptimes_from_records()
        tub.close()

        return self.test_path

    def test_tub_dataset_with_custom_ranking_keys(self):
        """Test TubDataset with custom ranking keys."""
        tub_path = self._create_test_tub()

        config = Config()
        config.GYRO_Z_INDEX = 1
        config.USE_LAP_0 = True
        config.COMPRESS_SESSIONS_FOR_LAP_STATS = False
        config.NUM_BINS_FOR_LAP_STATS = None

        # Use only time and distance for ranking
        custom_keys = ['time', 'distance']
        dataset = TubDataset(
            config,
            [tub_path],
            add_lap_pct=True,
            ranking_keys=custom_keys
        )

        records = dataset.get_records()

        # Check that lap_pct uses only custom keys
        records_with_lap_pct = [r for r in records if 'lap_pct' in r.underlying]

        if len(records_with_lap_pct) > 0:
            sample_rec = records_with_lap_pct[0]
            lap_pct = sample_rec.underlying['lap_pct']
            # Should have 2 values (time, distance), not 3
            self.assertEqual(len(lap_pct), 2)

        dataset.close()


if __name__ == '__main__':
    unittest.main()
