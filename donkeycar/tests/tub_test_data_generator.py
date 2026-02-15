"""
Shared data generator for segment ranking tests.

Provides reusable functions to create realistic multi-lap tub data
through the official Tub.write_record() and TubWriter.run() interfaces.
The module is NOT a test file — it contains no test classes or test
functions. It is imported by test files.

Sensor values are DERIVED from trajectory parameters using physics
equations. This guarantees physical consistency by construction.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from donkeycar.parts.tub_v2 import Tub, TubWriter
from donkeycar.parts.tub_statistics import (
    TubStatistics, FieldAggregationSpec,
)

# ── Minimal image to satisfy image_array type ──────────────────────
TINY_IMAGE = np.zeros((1, 1, 3), dtype=np.uint8)

# ── Full tub schema matching donkey5 template ──────────────────────
TUB_INPUTS = [
    'cam/image_array',
    'user/angle', 'user/throttle',
    'car/lap', 'car/segment',
    'car/gyro', 'car/accel', 'car/speed',
    'car/distance', 'car/pos', 'car/euler',
]
TUB_TYPES = [
    'image_array',
    'float', 'float',
    'int', 'int',
    'vector', 'vector', 'float',
    'float', 'vector', 'vector',
]

# ── Minimal schema for tests that don't need all fields ────────────
MINIMAL_INPUTS = [
    'car/lap', 'car/distance', 'car/gyro', 'car/accel', 'car/speed',
]
MINIMAL_TYPES = [
    'int', 'float', 'vector', 'vector', 'float',
]

# ── Realistic sensor ranges from real RC car hardware ──────────────
SENSOR_RANGES = {
    'speed_min': 0.0,
    'speed_max': 4.4,
    'gyro_min': -1.0,
    'gyro_max': 1.0,
    'accel_min': -1.0,
    'accel_max': 1.0,
    'dt_ms_min': 50,
    'dt_ms_max': 200,
    'distance_delta_max': 1.0,
}

# ── Physical constants for sensor derivation ───────────────────────
IMU_GYRO_NORM = 250.0     # °/s → normalized
IMU_ACCEL_NORM = 20.0     # m/s² → normalized


@dataclass
class DrivingProfile:
    """
    Parameterizes driving behavior for one segment using TRAJECTORY
    parameters. Sensor values are DERIVED from these using physics.
    """
    speed: float = 2.0
    speed_delta: float = 0.0
    curvature: float = 0.0
    smoothness: float = 0.9
    braking_profile: str = 'constant'
    braking_intensity: float = 0.0


# ── Pre-defined profiles ───────────────────────────────────────────

STRAIGHT_FAST = DrivingProfile(
    speed=3.5, curvature=0.0, smoothness=0.95,
)

STRAIGHT_SLOW = DrivingProfile(
    speed=1.5, curvature=0.0, smoothness=0.9,
)

TURN_SMOOTH = DrivingProfile(
    speed=2.0, curvature=0.5, smoothness=0.9,
)

TURN_AGGRESSIVE = DrivingProfile(
    speed=3.0, curvature=0.8, smoothness=0.6,
)

TURN_JERKY = DrivingProfile(
    speed=2.0, curvature=0.5, smoothness=0.3,
)

BRAKING_HARD = DrivingProfile(
    speed=3.0, speed_delta=-1.5, curvature=0.3,
    smoothness=0.8,
    braking_profile='brake_then_accel',
    braking_intensity=5.0,
)

BRAKING_GENTLE = DrivingProfile(
    speed=3.0, speed_delta=-0.5, curvature=0.3,
    smoothness=0.8,
    braking_profile='brake_then_accel',
    braking_intensity=1.5,
)

CHICANE = DrivingProfile(
    speed=2.0, curvature=0.0, smoothness=0.4,
)


class PhysicsViolation(Exception):
    """Raised when generated data violates physical constraints."""
    pass


def generate_sensor_record(
    profile: DrivingProfile,
    progress: float,
    record_index: int,
    dt_s: float = 0.1,
) -> dict:
    """
    Derive sensor values from trajectory parameters using physics.

    :param profile: Trajectory parameters
    :param progress: Position within segment (0.0 to 1.0)
    :param record_index: Global record index for deterministic noise
    :param dt_s: Time step in seconds
    :return: dict with keys: car/gyro, car/accel, car/speed
    """
    # Step 1: Compute instantaneous speed
    speed = profile.speed + progress * profile.speed_delta
    speed = max(speed, 0.0)

    # Noise factor from smoothness
    noise_factor = 1.0 - profile.smoothness

    # Step 2: Derive gyro_z from curvature and speed
    gyro_z_raw = speed * profile.curvature  # rad/s
    noise = noise_factor * 0.5 * math.sin(record_index * 7.3)
    gyro_z_raw += noise
    gyro_z_deg = gyro_z_raw * (180.0 / math.pi)
    gyro_z_norm = gyro_z_deg / IMU_GYRO_NORM
    gyro_z_norm = max(-1.0, min(1.0, gyro_z_norm))

    # Step 3: Derive gyro_x (roll) from lateral load
    gyro_x_raw = speed * speed * profile.curvature * 0.05
    gyro_x_norm = (gyro_x_raw * 180.0 / math.pi) / IMU_GYRO_NORM
    gyro_x_norm = max(-1.0, min(1.0, gyro_x_norm))

    gyro = [gyro_x_norm, 0.0, gyro_z_norm]

    # Step 4: Derive accel_x from speed profile
    if profile.braking_profile == 'constant':
        accel_x_raw = profile.speed_delta / (dt_s * 20)
    elif profile.braking_profile == 'brake_then_accel':
        if progress < 0.5:
            accel_x_raw = -profile.braking_intensity
        else:
            accel_x_raw = profile.braking_intensity * 0.3
    elif profile.braking_profile == 'accel_then_brake':
        if progress < 0.5:
            accel_x_raw = profile.braking_intensity * 0.3
        else:
            accel_x_raw = -profile.braking_intensity
    else:
        accel_x_raw = 0.0

    accel_x_raw += noise_factor * 0.3 * math.sin(record_index * 3.7)
    accel_x_norm = accel_x_raw / IMU_ACCEL_NORM
    accel_x_norm = max(-1.0, min(1.0, accel_x_norm))

    # Step 5: Derive accel_y from centripetal acceleration
    accel_y_raw = speed * speed * profile.curvature
    accel_y_raw += noise_factor * 0.5 * math.sin(record_index * 5.1)
    accel_y_norm = accel_y_raw / IMU_ACCEL_NORM
    accel_y_norm = max(-1.0, min(1.0, accel_y_norm))

    accel = [accel_x_norm, accel_y_norm, 0.0]

    return {
        'car/gyro': gyro,
        'car/accel': accel,
        'car/speed': speed,
    }


def validate_generated_tub(tub, strict=True) -> dict:
    """
    Validate that tub data is physically plausible.

    :param tub: Tub to validate
    :param strict: If True, raise PhysicsViolation on any failure.
    :return: dict with validation summary
    """
    violations = []
    speeds = []
    gyro_z_vals = []
    accel_x_vals = []
    accel_y_vals = []

    prev_speed = None
    prev_ts = None
    prev_dist = None
    prev_lap = None
    record_idx = 0

    # Per-segment accumulators for cross-channel checks
    seg_gyro_z = []
    seg_accel_y = []
    seg_accel_x = []
    seg_speeds = []
    current_lap = None
    current_segment = None

    def _check_segment_correlations():
        if not seg_gyro_z:
            return
        mean_gz = sum(abs(v) for v in seg_gyro_z) / len(seg_gyro_z)
        mean_ay = sum(abs(v) for v in seg_accel_y) / len(seg_accel_y)
        # Check 4: Gyro-curvature correlation
        if mean_gz > 0.05 and mean_ay < 0.01:
            violations.append(
                f"Turning (gyro_z={mean_gz:.4f}) without "
                f"lateral accel (accel_y={mean_ay:.4f})")
        if mean_ay > 0.05 and mean_gz < 0.01:
            violations.append(
                f"Lateral accel ({mean_ay:.4f}) without "
                f"turning ({mean_gz:.4f})")
        # Check 5: Accel-X vs speed change
        if len(seg_speeds) >= 2:
            mean_ax = sum(seg_accel_x) / len(seg_accel_x)
            speed_change = seg_speeds[-1] - seg_speeds[0]
            if speed_change > 0.5 and mean_ax < -0.1:
                violations.append(
                    f"Speed increasing ({speed_change:.2f}) but "
                    f"mean accel_x negative ({mean_ax:.4f})")
            if speed_change < -0.5 and mean_ax > 0.1:
                violations.append(
                    f"Speed decreasing ({speed_change:.2f}) but "
                    f"mean accel_x positive ({mean_ax:.4f})")

    for record in tub:
        speed = record.get('car/speed', 0.0)
        gyro = record.get('car/gyro', [0.0, 0.0, 0.0])
        accel = record.get('car/accel', [0.0, 0.0, 0.0])
        ts = record.get('_timestamp_ms', 0)
        dist = record.get('car/distance', 0.0)
        lap = record.get('car/lap', 0)
        segment = record.get('car/segment')

        speeds.append(speed)
        gyro_z_vals.append(gyro[2])
        accel_x_vals.append(accel[0])
        accel_y_vals.append(accel[1])

        # Check 1: Sensor range bounds
        if speed < SENSOR_RANGES['speed_min'] or \
                speed > SENSOR_RANGES['speed_max']:
            violations.append(f"Speed {speed} out of range")
        for i in range(3):
            if gyro[i] < SENSOR_RANGES['gyro_min'] or \
                    gyro[i] > SENSOR_RANGES['gyro_max']:
                violations.append(f"gyro[{i}]={gyro[i]} out of range")
            if accel[i] < SENSOR_RANGES['accel_min'] or \
                    accel[i] > SENSOR_RANGES['accel_max']:
                violations.append(f"accel[{i}]={accel[i]} out of range")

        # Check 2: Speed continuity (within same lap, skip segment
        # boundaries where speed changes are expected)
        segment_changed = (segment is not None
                           and current_segment is not None
                           and segment != current_segment)
        if prev_speed is not None and lap == prev_lap \
                and not segment_changed:
            dt_s = (ts - prev_ts) / 1000.0 if prev_ts else 0.1
            if dt_s > 0:
                speed_change = abs(speed - prev_speed)
                # Allow larger jumps at implicit segment boundaries
                max_change = 15.0 * dt_s + 0.5
                if speed_change > max_change:
                    violations.append(
                        f"Speed jump {speed_change:.2f} in {dt_s:.3f}s")

        # Check 3: Distance monotonicity
        if prev_dist is not None and lap == prev_lap:
            if dist < prev_dist - 0.001:
                violations.append(
                    f"Distance decreased: {prev_dist:.2f} -> {dist:.2f}")

        # Check 6: Timestamp spacing
        if prev_ts is not None:
            dt_ms = ts - prev_ts
            if dt_ms <= 0:
                violations.append(f"Timestamps not increasing: {dt_ms}ms")

        # Per-segment accumulation for cross-channel checks
        if lap != current_lap or segment != current_segment:
            _check_segment_correlations()
            seg_gyro_z = []
            seg_accel_y = []
            seg_accel_x = []
            seg_speeds = []
            current_lap = lap
            current_segment = segment

        seg_gyro_z.append(gyro[2])
        seg_accel_y.append(accel[1])
        seg_accel_x.append(accel[0])
        seg_speeds.append(speed)

        prev_speed = speed
        prev_ts = ts
        prev_dist = dist
        prev_lap = lap

    # Finalize last segment
    _check_segment_correlations()

    summary = {
        'num_records': len(speeds),
        'num_laps': (max(record.get('car/lap', 0) for record in tub) + 1)
        if speeds else 0,
        'speed_range': (min(speeds), max(speeds)) if speeds else (0, 0),
        'gyro_z_range': (min(gyro_z_vals), max(gyro_z_vals))
        if gyro_z_vals else (0, 0),
        'accel_x_range': (min(accel_x_vals), max(accel_x_vals))
        if accel_x_vals else (0, 0),
        'accel_y_range': (min(accel_y_vals), max(accel_y_vals))
        if accel_y_vals else (0, 0),
        'violations': violations,
        'valid': len(violations) == 0,
    }

    if strict and violations:
        raise PhysicsViolation(
            f"Generated data has {len(violations)} physics violations:\n"
            + "\n".join(f"  - {v}" for v in violations))

    return summary


def create_multilap_tub(
    tub_path: str,
    segment_profiles: List[List[DrivingProfile]],
    records_per_segment: int = 20,
    segment_time_ms: int = 2000,
    inputs: Optional[List[str]] = None,
    types: Optional[List[str]] = None,
) -> Tub:
    """
    Create a multi-lap tub with per-segment driving profiles.

    Records are written through Tub.write_record(). Laptimer metadata
    is generated automatically from the written data.

    :param tub_path: Directory path for the tub
    :param segment_profiles: Per-lap, per-segment driving profiles.
    :param records_per_segment: Number of records per segment
    :param segment_time_ms: Duration of each segment in milliseconds
    :param inputs: Tub input field names (default: MINIMAL_INPUTS)
    :param types: Tub input types (default: MINIMAL_TYPES)
    :return: Read-only Tub with laptimer metadata generated
    """
    if inputs is None:
        inputs = MINIMAL_INPUTS
    if types is None:
        types = MINIMAL_TYPES

    num_laps = len(segment_profiles)
    num_segments = len(segment_profiles[0])
    for lap_profiles in segment_profiles:
        assert len(lap_profiles) == num_segments, \
            "All laps must have the same number of segments"

    tub = Tub(tub_path, inputs, types)
    start_time_ms = 1000000
    cumulative_distance = 0.0
    global_record_idx = 0
    dt_ms = segment_time_ms // records_per_segment

    for lap_idx in range(num_laps):
        for seg_idx in range(num_segments):
            profile = segment_profiles[lap_idx][seg_idx]
            for rec_idx in range(records_per_segment):
                progress = rec_idx / max(records_per_segment, 1)
                timestamp = start_time_ms + global_record_idx * dt_ms

                sensor = generate_sensor_record(
                    profile, progress, global_record_idx,
                    dt_s=dt_ms / 1000.0)

                distance_step = (
                    profile.speed * segment_time_ms / 1000.0
                    / records_per_segment)
                cumulative_distance += distance_step

                record = {
                    'car/lap': lap_idx,
                    'car/distance': cumulative_distance,
                    'car/gyro': sensor['car/gyro'],
                    'car/accel': sensor['car/accel'],
                    'car/speed': sensor['car/speed'],
                    '_timestamp_ms': timestamp,
                }
                if 'car/segment' in inputs:
                    record['car/segment'] = seg_idx
                if 'cam/image_array' in inputs:
                    record['cam/image_array'] = TINY_IMAGE

                tub.write_record(record)
                global_record_idx += 1

    # Final record to close the last lap
    final_ts = start_time_ms + global_record_idx * dt_ms
    final_record = {
        'car/lap': num_laps,
        'car/distance': cumulative_distance,
        'car/gyro': [0.0, 0.0, 0.0],
        'car/accel': [0.0, 0.0, 0.0],
        'car/speed': 0.0,
        '_timestamp_ms': final_ts,
    }
    if 'car/segment' in inputs:
        final_record['car/segment'] = 0
    if 'cam/image_array' in inputs:
        final_record['cam/image_array'] = TINY_IMAGE
    tub.write_record(final_record)

    # Close and reopen to populate manifest_metadata sessions
    tub.close()
    tub = Tub(tub_path, read_only=True)

    # Generate laptimer metadata
    boundary_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
    ]
    stats = TubStatistics(tub, field_aggregations=boundary_aggs)
    stats.generate_laptimes_from_records()

    # Validate generated data
    validate_generated_tub(tub, strict=True)

    return tub


def create_tub_with_varied_segment_times(
    tub_path: str,
    num_laps: int,
    num_segments: int,
    lap_segment_times_ms: List[List[int]],
    base_profile: Optional[DrivingProfile] = None,
    profiles: Optional[List[List[DrivingProfile]]] = None,
    records_per_segment: int = 20,
    inputs: Optional[List[str]] = None,
    types: Optional[List[str]] = None,
) -> Tub:
    """
    Create tub where each segment instance has a specific duration.

    :param tub_path: Directory path for the tub
    :param num_laps: Number of laps
    :param num_segments: Number of segments per lap
    :param lap_segment_times_ms: Per-lap, per-segment duration in ms.
    :param base_profile: Profile for sensor values (default: TURN_SMOOTH)
    :param profiles: Optional per-lap, per-segment profiles
    :param records_per_segment: Records per segment
    :param inputs: Tub input field names
    :param types: Tub input types
    :return: The created Tub
    """
    if inputs is None:
        inputs = MINIMAL_INPUTS
    if types is None:
        types = MINIMAL_TYPES
    if base_profile is None:
        base_profile = TURN_SMOOTH

    tub = Tub(tub_path, inputs, types)
    start_time_ms = 1000000
    cumulative_distance = 0.0
    global_record_idx = 0
    cumulative_time_ms = 0

    for lap_idx in range(num_laps):
        for seg_idx in range(num_segments):
            seg_time_ms = lap_segment_times_ms[lap_idx][seg_idx]
            dt_ms = seg_time_ms // records_per_segment

            if profiles is not None:
                profile = profiles[lap_idx][seg_idx]
            else:
                profile = base_profile

            for rec_idx in range(records_per_segment):
                progress = rec_idx / max(records_per_segment, 1)
                timestamp = start_time_ms + cumulative_time_ms + \
                    rec_idx * dt_ms

                sensor = generate_sensor_record(
                    profile, progress, global_record_idx,
                    dt_s=dt_ms / 1000.0)

                distance_step = (
                    profile.speed * seg_time_ms / 1000.0
                    / records_per_segment)
                cumulative_distance += distance_step

                record = {
                    'car/lap': lap_idx,
                    'car/distance': cumulative_distance,
                    'car/gyro': sensor['car/gyro'],
                    'car/accel': sensor['car/accel'],
                    'car/speed': sensor['car/speed'],
                    '_timestamp_ms': timestamp,
                }
                if 'car/segment' in inputs:
                    record['car/segment'] = seg_idx
                if 'cam/image_array' in inputs:
                    record['cam/image_array'] = TINY_IMAGE

                tub.write_record(record)
                global_record_idx += 1

            cumulative_time_ms += seg_time_ms

    # Final record to close last lap
    final_ts = start_time_ms + cumulative_time_ms
    final_record = {
        'car/lap': num_laps,
        'car/distance': cumulative_distance,
        'car/gyro': [0.0, 0.0, 0.0],
        'car/accel': [0.0, 0.0, 0.0],
        'car/speed': 0.0,
        '_timestamp_ms': final_ts,
    }
    if 'car/segment' in inputs:
        final_record['car/segment'] = 0
    if 'cam/image_array' in inputs:
        final_record['cam/image_array'] = TINY_IMAGE
    tub.write_record(final_record)

    # Close and reopen to populate manifest_metadata sessions
    tub.close()
    tub = Tub(tub_path, read_only=True)

    # Generate laptimer metadata
    boundary_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
    ]
    stats = TubStatistics(tub, field_aggregations=boundary_aggs)
    stats.generate_laptimes_from_records()

    # Validate
    validate_generated_tub(tub, strict=True)

    return tub


def create_multilap_tub_via_writer(
    tub_path: str,
    segment_profiles: List[List[DrivingProfile]],
    records_per_segment: int = 20,
    segment_time_ms: int = 2000,
) -> Tub:
    """
    Same as create_multilap_tub but uses TubWriter.run() interface.

    Uses the full TUB_INPUTS/TUB_TYPES schema with images.
    Returns a read-only Tub.
    """
    num_laps = len(segment_profiles)
    num_segments = len(segment_profiles[0])

    writer = TubWriter(tub_path, TUB_INPUTS, TUB_TYPES)
    cumulative_distance = 0.0
    global_record_idx = 0

    for lap_idx in range(num_laps):
        for seg_idx in range(num_segments):
            profile = segment_profiles[lap_idx][seg_idx]
            for rec_idx in range(records_per_segment):
                progress = rec_idx / max(records_per_segment, 1)
                dt_s = segment_time_ms / 1000.0 / records_per_segment

                sensor = generate_sensor_record(
                    profile, progress, global_record_idx, dt_s=dt_s)

                distance_step = (
                    profile.speed * segment_time_ms / 1000.0
                    / records_per_segment)
                cumulative_distance += distance_step

                writer.run(
                    TINY_IMAGE,                # cam/image_array
                    0.0,                       # user/angle
                    0.5,                       # user/throttle
                    lap_idx,                   # car/lap
                    seg_idx,                   # car/segment
                    sensor['car/gyro'],        # car/gyro
                    sensor['car/accel'],       # car/accel
                    sensor['car/speed'],       # car/speed
                    cumulative_distance,       # car/distance
                    [0.0, 0.0, 0.0],          # car/pos
                    [0.0, 0.0, 0.0],          # car/euler
                )
                global_record_idx += 1

    # Final record to close last lap
    writer.run(
        TINY_IMAGE, 0.0, 0.5,
        num_laps, 0,
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0,
        cumulative_distance,
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
    )
    writer.close()

    # Reopen as read-only
    tub = Tub(tub_path, read_only=True)

    # Generate laptimer metadata
    boundary_aggs = [
        FieldAggregationSpec(output_key='time'),
        FieldAggregationSpec(output_key='distance'),
    ]
    stats = TubStatistics(tub, field_aggregations=boundary_aggs)
    stats.generate_laptimes_from_records()

    return tub
