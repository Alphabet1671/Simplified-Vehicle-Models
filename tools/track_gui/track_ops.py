"""Geometry-only track operations and safe exports for the track GUI."""
from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from tools.tracks.open_course import OpenCourse
from tools.track_gui.export import (SavedTrack, _guard_targets, _install, check_model_id,
                                    check_title, SaveRefused)
from tools.gui_common.files import check_name


@dataclass(frozen=True, slots=True)
class TrackData:
    distance: np.ndarray
    x_north: np.ndarray | None
    y_east: np.ndarray | None
    heading: np.ndarray
    curvature: np.ndarray
    is_closed: bool
    title: str
    maximum_radius: float = 300.0

    def __post_init__(self):
        for name in ("distance", "x_north", "y_east", "heading", "curvature"):
            value = getattr(self, name)
            if value is None:
                continue
            array = np.array(value, dtype=float, copy=True)
            array.setflags(write=False)
            object.__setattr__(self, name, array)
        n = len(self.distance)
        if self.distance.ndim != 1 or n < 2 or any(getattr(self, key).ndim != 1 or len(getattr(self, key)) != n for key in ("heading", "curvature")):
            raise ValueError("track arrays must be one-dimensional, equal length, and have at least two nodes")
        if not all(np.all(np.isfinite(getattr(self, key))) for key in ("distance", "heading", "curvature")):
            raise ValueError("track arrays must be finite")
        if not math.isclose(float(self.distance[0]), 0.0, abs_tol=1e-12) or np.any(np.diff(self.distance) <= 0):
            raise ValueError("distance must start at zero and increase")
        if (self.x_north is None) != (self.y_east is None):
            raise ValueError("both XY arrays must be present or absent")
        if self.x_north is not None and (self.x_north.ndim != 1 or self.y_east.ndim != 1 or len(self.x_north) != n or len(self.y_east) != n):
            raise ValueError("track arrays must be one-dimensional and have equal lengths")
        if self.x_north is not None and not (np.all(np.isfinite(self.x_north)) and np.all(np.isfinite(self.y_east))):
            raise ValueError("track arrays must be finite")

    @property
    def length(self) -> float:
        return float(self.distance[-1])


def load_track_data(path) -> TrackData:
    profile = load_track(path)
    coords = profile.path_coordinates
    xy = coords is not None
    return TrackData(profile.distance, coords.x_north if xy else None,
                     coords.y_east if xy else None, coords.heading if xy else np.zeros_like(profile.distance),
                     profile.curvature, profile.is_closed, profile.title)


def reverse_track(track: TrackData) -> TrackData:
    """Reverse travel direction, preserving the start point for closed XY tracks."""
    d = track.distance
    if track.x_north is None and track.is_closed:
        # Legacy CSV stores one radius per physical segment plus a terminal duplicate.
        if len(d) > 2 and np.isclose(track.curvature[-1], track.curvature[0], rtol=0, atol=1e-12):
            values = np.r_[track.curvature[:-1][::-1], track.curvature[0]]
        else:
            values = track.curvature[::-1]
        nd = np.r_[0.0, np.cumsum(np.diff(d)[::-1])]
        return TrackData(nd, None, None, np.zeros_like(nd), values, track.is_closed, track.title, track.maximum_radius)
    if track.is_closed and track.x_north is not None:
        idx = np.r_[0, np.arange(len(d)-2, 0, -1), 0]
        seg = np.diff(d)[::-1]
        nd = np.r_[0.0, np.cumsum(seg)]
        hd = np.unwrap(track.heading[idx] + math.pi)
        curv = -track.curvature[idx]
        curv[-1] = curv[0]
        return TrackData(nd, track.x_north[idx], track.y_east[idx], hd, curv, True, track.title, track.maximum_radius)
    idx = np.arange(len(d)-1, -1, -1)
    lengths = np.diff(d)[::-1]
    nd = np.r_[0.0, np.cumsum(lengths)]
    if track.x_north is None:
        return TrackData(nd, None, None, np.zeros_like(nd), track.curvature[idx], track.is_closed, track.title, track.maximum_radius)
    hd = np.unwrap(track.heading[idx] + math.pi)
    return TrackData(nd, track.x_north[idx], track.y_east[idx], hd, -track.curvature[idx], track.is_closed, track.title, track.maximum_radius)


def _periodic_samples(track: TrackData, cuts: list[float]):
    if track.x_north is None:
        raise ValueError("section export requires signed XY geometry")
    try:
        from scipy.interpolate import CubicSpline
    except ImportError as exc:
        raise ValueError("SciPy is required to reconstruct strict open sections; install the forward extra") from exc
    length = track.length
    if length <= 0 or not track.is_closed:
        raise ValueError("section export requires a closed track")
    # A periodic spline uses unique nodes; preserve explicit endpoints as the seam.
    source_d = track.distance
    if not np.allclose(track.x_north[0], track.x_north[-1], atol=1e-8) or not np.allclose(track.y_east[0], track.y_east[-1], atol=1e-8):
        raise ValueError("closed XY geometry must repeat its start point")
    sx = CubicSpline(source_d, track.x_north, bc_type="periodic")
    sy = CubicSpline(source_d, track.y_east, bc_type="periodic")
    if any(not math.isfinite(float(c)) for c in cuts):
        raise ValueError("cut distances must be finite")
    values = [float(c) for c in cuts]
    if any(c < 0.0 or c > length for c in values):
        raise ValueError("cut distances must lie between zero and track length")
    normalized = sorted(set((0.0 if math.isclose(c, length, abs_tol=1e-10) else c) for c in values))
    if len(normalized) < 2:
        raise ValueError("at least two distinct section cuts are required")
    output=[]
    for start, end0 in zip(normalized, normalized[1:]+[normalized[0]+length]):
        end=end0
        # Source-parametric dense samples, with exact endpoints.
        count=max(2, int(math.ceil((end-start)/0.1)))
        u=np.linspace(start,end,count+1)
        q=u % length
        q[-1] = end % length
        # Evaluate wrapped positions; spline accepts periodic extended domain by wrapping.
        n=sx(q); e=sy(q)
        dn=sx(q,1); de=sy(q,1); ddn=sx(q,2); dde=sy(q,2)
        speed=np.hypot(dn,de)
        ds=np.diff(u)*(speed[:-1]+speed[1:])*0.5
        dist=np.r_[0,np.cumsum(ds)]
        heading=np.unwrap(np.arctan2(de,dn))
        curv=(dn*dde-de*ddn)/np.maximum(speed,1e-12)**3
        # Endpoint periodic cut may have a discontinuous unwrapped tangent; match direction continuation.
        heading[-1]=heading[-2]+((heading[-1]-heading[-2]+np.pi)%(2*np.pi)-np.pi)
        output.append(OpenCourse(dist,n,e,heading,curv,(0.0,float(dist[-1])),("section",),()))
    return output


def split_closed(track: TrackData, cuts) -> list[TrackData]:
    courses = _periodic_samples(track, list(cuts))
    # Geometry is already sampled from smooth XY; return an intermediate TrackData list.
    return [TrackData(c.distance,c.x_north,c.y_east,c.heading,c.curvature,False,track.title,track.maximum_radius) for c in courses]


def _write_track(track, folder: Path, stem: str, model_id: str, title: str, provenance: str):
    from tools.tracks.open_course import csv_text, xml_text
    course=OpenCourse(track.distance,track.x_north,track.y_east,track.heading,track.curvature,(0.,track.length),("section",),())
    (folder/f"{stem}.csv").write_text(csv_text(course),encoding="utf-8")
    (folder/f"{stem}.xml").write_text(xml_text(course,model_id,title,f"{stem}.csv",[provenance]),encoding="utf-8")


def _save_many(tracks, directory, stems, model_id, title, *, allow_models, overwrite, overwrite_generated):
    directory=Path(directory); check_model_id(model_id); check_title(title)
    for stem in stems:
        check_name(stem,"file name")
        _guard_targets(directory,stem,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated)
    directory.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as temp:
        scratch=Path(temp)
        for i,(track,stem) in enumerate(zip(tracks,stems),1):
            _write_track(track,scratch,stem,model_id,title,f"Section {i}; reconstructed from periodic cubic XY spline.")
            loaded=load_track(scratch/f"{stem}.xml")
            if loaded.is_closed: raise ValueError("section did not load as open")
        # Install as one rollback transaction. Back up all existing targets first.
        backups=[]; installed=[]
        try:
            for stem in stems:
                for ext in (".csv",".xml"):
                    target=directory/f"{stem}{ext}"
                    if target.exists():
                        backup=scratch/f"backup-{len(backups)}"
                        import os; os.replace(target,backup); backups.append((target,backup))
            import os
            for stem in stems:
                for ext in (".csv",".xml"):
                    target=directory/f"{stem}{ext}"
                    os.replace(scratch/f"{stem}{ext}",target); installed.append(target)
        except Exception:
            for target in installed:
                try: target.unlink()
                except FileNotFoundError: pass
            for target,backup in backups:
                if backup.exists(): os.replace(backup,target)
            raise
    result=[]
    for stem in stems:
        p=load_track(directory/f"{stem}.xml")
        result.append(SavedTrack(directory/f"{stem}.xml",directory/f"{stem}.csv",float(p.lap_length),False,len(p.distance)))
    return result


def save_track_data(track, directory, stem, model_id, title, *, allow_models=False, overwrite=False, overwrite_generated=False):
    return _save_many([track],directory,[stem],model_id,title,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated)[0]


def save_sections(track, cuts, directory, stem, model_id, title, *, allow_models=False, overwrite=False, overwrite_generated=False):
    parts=split_closed(track,cuts)
    stems=[f"{stem}_{i:02d}" for i in range(1,len(parts)+1)]
    return _save_many(parts,directory,stems,model_id,title,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated)
