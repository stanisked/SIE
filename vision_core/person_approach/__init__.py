"""Dry-run stop-and-measure person-approach decisions, without actuation."""

from .decision import PersonApproachDecision, PersonApproachDecisionEngine
from .bounded_bridge import BoundedCommandBlock, PlannedBoundedCommand, plan_bounded_command
from .supervised_demo import SupervisedPersonApproachDemo
from .far_field_alignment import FarFieldAlignmentResult, evaluate_far_field_alignment
from .temporal_yaw_alignment import TemporalYawAlignmentPlanner, TemporalYawAlignmentResult, evaluate_temporal_yaw_alignment
from .supervised_acquire_range import SupervisedAcquireRangeResult, coordinate_supervised_acquire_range
from .supervised_target_dry_run import SupervisedTargetDryRun, SupervisedTargetResult

__all__ = [
    "BoundedCommandBlock",
    "PersonApproachDecision",
    "PersonApproachDecisionEngine",
    "PlannedBoundedCommand",
    "SupervisedPersonApproachDemo",
    "FarFieldAlignmentResult",
    "evaluate_far_field_alignment",
    "TemporalYawAlignmentPlanner",
    "TemporalYawAlignmentResult",
    "evaluate_temporal_yaw_alignment",
    "SupervisedAcquireRangeResult",
    "coordinate_supervised_acquire_range",
    "SupervisedTargetDryRun",
    "SupervisedTargetResult",
    "plan_bounded_command",
]
