"""
Course Analysis Package

Refactored architecture for IMU path visualization and course segmentation.

Modules:
    data_loader: Data loading (CSV, Tub) and PathData container
    lap_detection: Lap boundary detection algorithms (Phase 2)
    mean_course: Mean course reconstruction (Phase 3)
    segmentation: Course segmentation strategies (Phase 4)
    segment_assignment: Segment assignment to driven paths (Phase 5)

Design principles:
- Pure functions where possible (no hidden state)
- Dependency injection (swap algorithms)
- Strategy pattern (pluggable algorithms)
- Immutable data containers
- Configuration via params dict
- No call-order dependencies

Phase 1 exports: Data loading
Phase 2 exports: Lap detection
Phase 3 exports: Mean course
"""

# Phase 1: Data loading
from .data_loader import (
    PathData,
    PathDataSource,
    CSVPathDataSource,
    TubPathDataSource,
)

# Phase 2: Lap detection
from .lap_detection import (
    LapBoundary,
    LapDetector,
    YCrossingLapDetector,
    DriftLapDetector,
    MultiLapData,
)

# Phase 3: Mean course
from .mean_course import (
    MeanCourse,
    MeanCourseBuilder,
)

# Phase 4: Segmentation
from .segmentation import (
    SegmentType,
    Segment,
    SegmentationStrategy,
    ThresholdSegmentation,
    ExtremaSegmentation,
    GradientSegmentation,
    HybridSegmentation,
    CourseSegmenter,
    CourseSegmentation,
)

# Phase 5: Segment assignment
from .segment_assignment import (
    SegmentEstimate,
    SegmentAssigner,
    SegmentEstimator,
)

# Segment reconstruction utilities
from .segment_reconstruction import (
    reconstruct_segment_assigner,
    compute_segment_id,
    get_or_compute_segment_id,
)

# Utility functions
from .utils import (
    normalize_angle,
    angle_difference,
    circular_mean,
    circular_std,
)

__all__ = [
    # Data loading
    'PathData',
    'PathDataSource',
    'CSVPathDataSource',
    'TubPathDataSource',
    # Lap detection
    'LapBoundary',
    'LapDetector',
    'YCrossingLapDetector',
    'DriftLapDetector',
    'MultiLapData',
    # Mean course
    'MeanCourse',
    'MeanCourseBuilder',
    # Segmentation
    'SegmentType',
    'Segment',
    'SegmentationStrategy',
    'ThresholdSegmentation',
    'ExtremaSegmentation',
    'GradientSegmentation',
    'HybridSegmentation',
    'CourseSegmenter',
    'CourseSegmentation',
    # Segment assignment
    'SegmentEstimate',
    'SegmentAssigner',
    'SegmentEstimator',
    # Segment reconstruction utilities
    'reconstruct_segment_assigner',
    'compute_segment_id',
    'get_or_compute_segment_id',
    # Utility functions
    'normalize_angle',
    'angle_difference',
    'circular_mean',
    'circular_std',
]
