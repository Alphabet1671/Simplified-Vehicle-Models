"""Deterministic display sweeps, independent of objective/measurement sampling."""
import numpy as np

from .contracts import SampleSet


def model_sweeps(samples: SampleSet, variable: str, count: int = 121) -> tuple[SampleSet, ...]:
    """Sample each segment/load separately, holding the other slip constant.

    Compatibility samples carry nominal loads and fixed secondary slip per
    segment. Medians suppress acquisition jitter for display only; objective
    predictions and residuals must still use the original samples.
    """
    if variable not in {'alpha', 'kappa'}:
        raise ValueError('sweep variable must be alpha or kappa')
    if count < 2:
        raise ValueError('sweeps need at least two points')
    conditions = dict.fromkeys(zip(samples.segment_ids.tolist(), samples.fz.tolist()))
    result = []
    for segment, load in conditions:
        mask = (samples.segment_ids == segment) & (samples.fz == load)
        varying = getattr(samples, variable)[mask]
        grid = np.linspace(np.min(varying), np.max(varying), count)
        alpha = grid if variable == 'alpha' else np.full(count, np.median(samples.alpha[mask]))
        kappa = grid if variable == 'kappa' else np.full(count, np.median(samples.kappa[mask]))
        result.append(SampleSet(alpha, kappa, np.full(count, load), np.zeros(count),
                                np.full(count, segment), samples.run_id, samples.tire_id))
    return tuple(result)
