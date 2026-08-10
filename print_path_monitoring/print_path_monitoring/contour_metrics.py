"""Robust one-dimensional laser-profile comparison for monitoring only."""

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class ContourError:
    valid: bool
    lateral_error_m: float = 0.0
    height_error_m: float = 0.0
    overlap: int = 0
    reason: str = ''


def estimate_contour_error(reference_z, observed_z, x_pitch_m: float,
                           max_shift_samples: int = 20, min_overlap: int = 12) -> ContourError:
    """Estimate observed contour displacement against a reference profile.

    Positive lateral error means an observed feature is at a higher X than its
    matching reference feature. Positive height error means the observed profile
    is higher. Invalid returns must be NaN and are ignored; insufficient overlap
    and a laterally unobservable flat reference fail closed.
    """
    if not math.isfinite(x_pitch_m) or x_pitch_m <= 0.0:
        return ContourError(False, reason='invalid profile pitch')
    if len(reference_z) != len(observed_z):
        return ContourError(False, reason='reference and observed lengths differ')
    if len(reference_z) < min_overlap:
        return ContourError(False, reason='profile shorter than required overlap')
    candidates = []
    for shift in range(-max_shift_samples, max_shift_samples + 1):
        pairs = [(float(reference_z[index]), float(observed_z[index + shift]))
                 for index in range(len(reference_z))
                 if 0 <= index + shift < len(observed_z)
                 and math.isfinite(float(reference_z[index]))
                 and math.isfinite(float(observed_z[index + shift]))]
        if len(pairs) < min_overlap:
            continue
        ref_values = [pair[0] for pair in pairs]
        if max(ref_values) - min(ref_values) < 1e-6:
            continue
        height = statistics.median(observed - reference for reference, observed in pairs)
        residual = sum((observed - reference - height) ** 2 for reference, observed in pairs) / len(pairs)
        candidates.append((residual, abs(shift), shift, height, len(pairs)))
    if not candidates:
        return ContourError(False, reason='insufficient finite, laterally observable profile overlap')
    _residual, _abs_shift, shift, height, overlap = min(candidates)
    return ContourError(True, shift * x_pitch_m, height, overlap)
