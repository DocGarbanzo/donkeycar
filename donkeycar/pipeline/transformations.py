"""
Sorting utilities for tub data processing.

Simple, functional approach to sorting lap data by configurable criteria.
Uses plain Python functions/lambdas - no unnecessary classes.
"""

from typing import Callable, Any, List, Optional, Dict
import logging

logger = logging.getLogger(__name__)


# Simple utility functions (not wrapped in classes)
def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value to [min_val, max_val]."""
    return max(min_val, min(max_val, value))


class SortingStrategy:
    """
    Defines how to sort and rank laps.
    
    Uses simple dictionaries to configure sorting criteria:
    {
        'key': 'time',  # field name in lap data
        'transform': lambda x: x,  # optional transformation function
        'reverse': False  # True for descending
    }
    """

    def __init__(self, criteria: List[Dict[str, Any]]):
        """
        Initialize sorting strategy.

        :param criteria: List of criterion dicts with:
                        - 'key': field name (required)
                        - 'transform': function to apply (optional, default identity)
                        - 'reverse': sort descending (optional, default False)
        """
        self.criteria = []
        for spec in criteria:
            criterion = {
                'key': spec['key'],
                'transform': spec.get('transform', lambda x: x),
                'reverse': spec.get('reverse', False)
            }
            self.criteria.append(criterion)

    def rank_laps(self, laps: List[dict], num_buckets: Optional[int] = None) -> Dict[int, Dict[str, float]]:
        """
        Rank laps according to sorting criteria.

        Returns {lap_index: {criterion_key: ranking_value}}
        where ranking_value is between 0 and 1.

        :param laps: List of lap dictionaries to rank
        :param num_buckets: Optional number of buckets for quantization
        :return: Dictionary of rankings
        """
        num_laps = len(laps)
        if num_laps == 0:
            return {}

        num_buckets = num_buckets or num_laps
        rankings = {i: {} for i in range(len(laps))}

        for criterion in self.criteria:
            key = criterion['key']
            transform = criterion['transform']
            reverse = criterion['reverse']

            try:
                # Sort laps by this criterion
                indexed_laps = [(i, lap) for i, lap in enumerate(laps)]
                
                def get_sort_value(item):
                    _, lap = item
                    value = lap.get(key)
                    if value is None:
                        return float('inf')
                    return transform(value)
                
                sorted_laps = sorted(indexed_laps, key=get_sort_value, reverse=reverse)

                # Assign quantile rankings
                for rank_idx, (orig_idx, lap) in enumerate(sorted_laps):
                    rel_rank = int(rank_idx * num_buckets / num_laps + 1) / num_buckets
                    rankings[orig_idx][key] = rel_rank

                # Log statistics
                values = [get_sort_value(item) for item in sorted_laps]
                min_val = values[0] if values[0] != float('inf') else None
                max_val = values[-1] if values[-1] != float('inf') else None

                if min_val is not None and max_val is not None:
                    logger.info(
                        f'Sorted {num_laps} laps by {key}: '
                        f'min={min_val:.2f}, max={max_val:.2f}'
                    )
            except Exception as e:
                logger.warning(f'Failed to rank by {key}: {e}')
                # Assign default ranking if extraction fails
                for i in range(len(laps)):
                    rankings[i][key] = 0.5

        return rankings


def default_lap_sorting_strategy() -> SortingStrategy:
    """
    Create default lap sorting strategy.

    Sorts by: time and distance (all ascending).
    Only includes fields that are always computed from lap timing.

    Note: For custom field rankings (e.g., gyro_z_agg), configure
    LAP_SORTING_CRITERIA in your config file.
    """
    return SortingStrategy([
        {'key': 'time'},
        {'key': 'distance'},
    ])
