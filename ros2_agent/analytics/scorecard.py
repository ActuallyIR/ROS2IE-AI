"""North-star metrics helpers for weekly scorecards."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


@dataclass
class ScorecardInput:
    """Raw product and GTM metrics for a single reporting window."""

    stars_per_week: float = 0.0
    clone_to_demo_rate: float = 0.0
    median_diagnosis_minutes: float = 60.0
    replay_share_rate: float = 0.0
    pilot_setup_hours: float = 24.0
    security_checklist_pass_rate: float = 0.0


def build_weekly_scorecard(metrics: ScorecardInput) -> dict:
    """Convert raw metrics into a weighted impact scorecard."""
    adoption = (
        0.6 * _clamp01(metrics.stars_per_week / 60.0)
        + 0.4 * _clamp01(metrics.clone_to_demo_rate)
    ) * 100.0
    product_value = (
        0.55 * _clamp01((45.0 - metrics.median_diagnosis_minutes) / 45.0)
        + 0.45 * _clamp01(metrics.replay_share_rate)
    ) * 100.0
    commercial_readiness = (
        0.5 * _clamp01((24.0 - metrics.pilot_setup_hours) / 24.0)
        + 0.5 * _clamp01(metrics.security_checklist_pass_rate)
    ) * 100.0
    composite = 0.35 * adoption + 0.35 * product_value + 0.30 * commercial_readiness

    return {
        "window_utc": datetime.now(UTC).isoformat(),
        "inputs": {
            "stars_per_week": round(metrics.stars_per_week, 3),
            "clone_to_demo_rate": round(metrics.clone_to_demo_rate, 3),
            "median_diagnosis_minutes": round(metrics.median_diagnosis_minutes, 3),
            "replay_share_rate": round(metrics.replay_share_rate, 3),
            "pilot_setup_hours": round(metrics.pilot_setup_hours, 3),
            "security_checklist_pass_rate": round(metrics.security_checklist_pass_rate, 3),
        },
        "scores": {
            "adoption": round(adoption, 2),
            "product_value": round(product_value, 2),
            "commercial_readiness": round(commercial_readiness, 2),
            "composite": round(composite, 2),
        },
    }
