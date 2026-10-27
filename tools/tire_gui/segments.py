import numpy as np

from .contracts import RunData, SampleSet, SteadySelection

RUN8 = [(1,2493,0,1112),(2494,3739,0,889),(3740,4981,0,660),(4982,6229,0,222),(6230,7474,0,1112),(7475,8716,0,445),(8717,9959,2,889),(9960,11207,2,660),(11208,12452,2,222),(12453,13699,2,1112),(13700,14944,2,445),(14945,16189,4,889),(16190,17424,4,660),(17425,18677,4,222),(18678,19920,4,1112),(19921,21168,4,445)]
RUN72_SA0 = [(383,775,0,1112),(1255,1972,0,889),(2308,2552,0,660),(3283,3927,0,222),(5830,6060,2,1112),(3928,4645,2,889),(4966,5226,2,660),(6061,6709,2,222),(8412,8843,4,1112),(6710,7421,4,889),(7592,8023,4,660),(8844,9499,4,222)]
RUN72_COMB = [(9500,10081,0,889,-3),(10210,10917,0,660,-3),(10918,11604,0,1112,-3),(11605,12251,0,222,-3),(12252,12965,2,889,-3),(12966,13683,2,660,-3),(13684,14412,2,1112,-3),(14413,15064,2,222,-3),(15065,15761,4,889,-3),(15762,16475,4,660,-3),(16745,17189,4,1112,-3),(17190,17834,4,222,-3),(17835,18544,0,889,-6),(18545,19172,0,660,-6),(19173,19865,0,1112,-6),(19866,20524,0,222,-6),(20525,21234,2,889,-6),(21235,21895,2,660,-6),(21896,22596,2,1112,-6),(22597,23249,2,222,-6),(23250,23960,4,889,-6),(23961,24616,4,660,-6),(24617,25336,4,1112,-6),(25337,25992,4,222,-6)]


def matlab_segment_indices(start: int, end: int, maximum_count: int = 180) -> np.ndarray:
    count = end - start + 1
    if start < 1 or count <= 0 or maximum_count < 2:
        raise ValueError('invalid one-based inclusive segment')
    values = np.arange(start, end + 1) if count <= maximum_count else np.unique(np.floor(np.linspace(start, end, maximum_count) + .5))
    return values.astype(np.int64) - 1


def _samples(run: RunData, rows, target_key: str) -> SampleSet:
    channel = run.channels
    required = ('alpha', 'kappa', 'fz', target_key)
    if any(name not in channel for name in required):
        raise ValueError(f"run {run.run_id} lacks required channels: {required}")
    selected_rows = [row for row in rows if row[2] == 0]
    ids, raws, alphas, kappas, loads, targets = [], [], [], [], [], []
    for seg, row in enumerate(selected_rows):
        inds = matlab_segment_indices(row[0], row[1], 180)
        channel_lengths = {len(v) for v in run.channels.values()}
        if len(channel_lengths) != 1 or np.max(inds) >= len(run.raw_indices) or np.max(inds) >= next(iter(channel_lengths)):
            raise ValueError(f"run {run.run_id} is too short or has mismatched channels for source compatibility segment ending {row[1]}")
        n = len(inds)
        raws.extend(run.raw_indices[inds].tolist()); ids.extend([seg] * n)
        if len(row) > 4:
            alphas.extend([-np.deg2rad(float(row[4]))] * n)
            kappas.extend(channel['kappa'][inds].tolist())
        elif run.run_id == 72:
            alphas.extend([0.0] * n); kappas.extend(channel['kappa'][inds].tolist())
        else:
            alphas.extend(channel['alpha'][inds].tolist()); kappas.extend([0.0] * n)
        loads.extend([float(row[3])] * n)
        targets.extend(channel[target_key][inds].tolist())
    return SampleSet(np.asarray(alphas), np.asarray(kappas), np.asarray(loads), np.asarray(targets), np.asarray(ids), run.run_id, run.tire_id, np.asarray(raws))


def compatibility_selection(lateral: RunData, longitudinal: RunData) -> SteadySelection:
    if lateral.run_id != 8 or longitudinal.run_id != 72:
        raise ValueError('compatibility selection requires run 8 lateral and run 72 longitudinal')
    return SteadySelection(
        _samples(lateral, RUN8, 'fy'), _samples(longitudinal, RUN72_SA0, 'fx'),
        _samples(longitudinal, RUN72_COMB, 'fx'), _samples(longitudinal, RUN72_COMB, 'fy'),
        _samples(lateral, RUN8, 'mz'), 40.193 / 3.6)
