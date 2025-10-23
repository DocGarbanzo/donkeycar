"""
Data transformation and sorting utilities for tub data processing.

This module provides modular transformation and sorting capabilities for
donkeycar data pipelines, replacing hardcoded transformations with flexible,
composable functions.
"""

from typing import Callable, Any, List, Optional, Union
import logging

logger = logging.getLogger(__name__)


class Transformation:
    """
    A transformation function that can be applied to data values.

    Transformations are composable and can be chained together.
    Common transformations include abs(), clamp(), normalize(), etc.
    """

    def __init__(self, func: Callable[[Any], Any], name: Optional[str] = None):
        """
        Initialize a transformation.

        :param func: Function to apply to values
        :param name: Optional name for the transformation (for logging)
        """
        self.func = func
        self.name = name or func.__name__

    def __call__(self, value: Any) -> Any:
        """Apply the transformation to a value."""
        return self.func(value)

    def __repr__(self) -> str:
        return f"Transformation({self.name})"

    def compose(self, other: 'Transformation') -> 'Transformation':
        """
        Compose this transformation with another.

        Returns a new transformation that applies this transformation first,
        then the other transformation.

        :param other: Another transformation to compose with
        :return: Composed transformation
        """
        def composed(value):
            return other(self(value))

        return Transformation(
            composed,
            name=f"{self.name} -> {other.name}"
        )


# Common transformations
def abs_transform() -> Transformation:
    """Create an absolute value transformation."""
    return Transformation(abs, name="abs")


def clamp_transform(min_val: float, max_val: float) -> Transformation:
    """
    Create a clamp transformation that limits values to [min_val, max_val].

    :param min_val: Minimum value
    :param max_val: Maximum value
    :return: Clamp transformation
    """
    def clamp(value):
        return max(min_val, min(max_val, value))

    return Transformation(clamp, name=f"clamp({min_val}, {max_val})")


def scale_transform(factor: float) -> Transformation:
    """
    Create a scaling transformation that multiplies values by a factor.

    :param factor: Scale factor
    :return: Scale transformation
    """
    return Transformation(lambda x: x * factor, name=f"scale({factor})")


def offset_transform(offset: float) -> Transformation:
    """
    Create an offset transformation that adds a constant to values.

    :param offset: Offset value
    :return: Offset transformation
    """
    return Transformation(lambda x: x + offset, name=f"offset({offset})")


def normalize_transform(min_val: float, max_val: float) -> Transformation:
    """
    Create a normalization transformation that maps [min_val, max_val] to [0, 1].

    :param min_val: Minimum value of input range
    :param max_val: Maximum value of input range
    :return: Normalize transformation
    """
    def normalize(value):
        if max_val == min_val:
            return 0.0
        return (value - min_val) / (max_val - min_val)

    return Transformation(normalize, name=f"normalize({min_val}, {max_val})")


def identity_transform() -> Transformation:
    """Create an identity transformation (returns value unchanged)."""
    return Transformation(lambda x: x, name="identity")


class SortingCriterion:
    """
    A sorting criterion that defines how to extract and sort values from data.

    This replaces hardcoded sorting by 'time', 'distance', 'gyro_z_agg' with
    a flexible system that allows any field or computed value to be used for sorting.
    """

    def __init__(self,
                 key: str,
                 extractor: Optional[Callable[[dict], Any]] = None,
                 transformation: Optional[Transformation] = None,
                 reverse: bool = False):
        """
        Initialize a sorting criterion.

        :param key: Name of the criterion (e.g., 'time', 'distance', 'custom')
        :param extractor: Optional function to extract value from data dict.
                         If None, uses dict[key]
        :param transformation: Optional transformation to apply before sorting
        :param reverse: If True, sort in descending order
        """
        self.key = key
        self.extractor = extractor or (lambda data: data.get(key))
        self.transformation = transformation or identity_transform()
        self.reverse = reverse

    def get_sort_value(self, data: dict) -> Any:
        """
        Extract and transform the sort value from data.

        :param data: Data dictionary
        :return: Transformed sort value
        """
        value = self.extractor(data)
        return self.transformation(value) if value is not None else float('inf')

    def __repr__(self) -> str:
        return f"SortingCriterion(key='{self.key}', transform={self.transformation}, reverse={self.reverse})"


class SortingStrategy:
    """
    A sorting strategy that defines multiple criteria for ranking laps.

    This replaces the hardcoded rank_laps nested function with a modular,
    configurable approach.
    """

    def __init__(self, criteria: List[SortingCriterion]):
        """
        Initialize a sorting strategy.

        :param criteria: List of sorting criteria to apply
        """
        self.criteria = criteria

    def rank_laps(self, laps: List[dict], num_buckets: Optional[int] = None) -> dict:
        """
        Rank laps according to the sorting criteria.

        Returns a dictionary of {lap_index: {criterion_key: ranking_value}}
        where ranking_value is between 0 and 1.

        :param laps: List of lap dictionaries to rank
        :param num_buckets: Optional number of buckets for quantization
        :return: Dictionary of rankings
        """
        num_laps = len(laps)
        if num_laps == 0:
            return {}

        num_buckets = num_buckets or num_laps

        # Initialize rankings dictionary
        rankings = {i: {} for i in range(len(laps))}

        # Rank by each criterion
        for criterion in self.criteria:
            try:
                # Sort laps by this criterion
                indexed_laps = [(i, lap) for i, lap in enumerate(laps)]
                sorted_laps = sorted(
                    indexed_laps,
                    key=lambda x: criterion.get_sort_value(x[1]),
                    reverse=criterion.reverse
                )

                # Assign quantile rankings
                for rank_idx, (orig_idx, lap) in enumerate(sorted_laps):
                    rel_rank = int(rank_idx * num_buckets / num_laps + 1) / num_buckets
                    rankings[orig_idx][criterion.key] = rel_rank

                # Log statistics
                if num_laps > 0:
                    values = [criterion.get_sort_value(lap) for _, lap in sorted_laps]
                    min_val = values[0] if values[0] != float('inf') else None
                    max_val = values[-1] if values[-1] != float('inf') else None

                    if min_val is not None and max_val is not None:
                        logger.info(
                            f'Sorted {num_laps} laps by {criterion.key}: '
                            f'min={min_val:.2f}, max={max_val:.2f}'
                        )
            except Exception as e:
                logger.warning(f'Failed to rank by {criterion.key}: {e}')
                # Assign default ranking of 0.5 if extraction fails
                for i in range(len(laps)):
                    rankings[i][criterion.key] = 0.5

        return rankings

    def __repr__(self) -> str:
        criteria_str = ', '.join(str(c) for c in self.criteria)
        return f"SortingStrategy([{criteria_str}])"


# Default sorting strategies for backward compatibility
def default_lap_sorting_strategy() -> SortingStrategy:
    """
    Create the default lap sorting strategy used in original implementation.

    Sorts by: time, distance, gyro_z_agg (all ascending)
    """
    return SortingStrategy([
        SortingCriterion('time'),
        SortingCriterion('distance'),
        SortingCriterion('gyro_z_agg'),
    ])


def custom_lap_sorting_strategy(criteria_specs: List[dict]) -> SortingStrategy:
    """
    Create a custom sorting strategy from specifications.

    :param criteria_specs: List of criterion specifications, each a dict with:
                          - key: str (criterion name)
                          - extractor: Optional[Callable] (extraction function)
                          - transformation: Optional[str or Transformation]
                          - reverse: Optional[bool]
    :return: Configured sorting strategy
    """
    criteria = []
    for spec in criteria_specs:
        # Parse transformation if it's a string
        transform = spec.get('transformation')
        if isinstance(transform, str):
            if transform == 'abs':
                transform = abs_transform()
            elif transform == 'identity':
                transform = identity_transform()
            elif transform.startswith('clamp'):
                # Parse clamp(min, max)
                import re
                match = re.match(r'clamp\(([^,]+),\s*([^)]+)\)', transform)
                if match:
                    min_val, max_val = float(match.group(1)), float(match.group(2))
                    transform = clamp_transform(min_val, max_val)
                else:
                    transform = None
            else:
                transform = None

        criterion = SortingCriterion(
            key=spec['key'],
            extractor=spec.get('extractor'),
            transformation=transform,
            reverse=spec.get('reverse', False)
        )
        criteria.append(criterion)

    return SortingStrategy(criteria)
