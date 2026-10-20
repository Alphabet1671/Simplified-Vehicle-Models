"""Geometry-only track operations and safe exports for the track GUI."""
from __future__ import annotations

import csv
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from tools.tracks.open_course import OpenCourse, csv_text, xml_text
from tools.track_gui.export import SavedTrack, _guard_targets, check_model_id, check_title, SaveRefused
from tools.gui_common.files import check_name

MAX_SECTION_NODES = 100_000
MAX_RECONSTRUCTION_DEPARTURE_M = 0.02

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
            value=getattr(self,name)
            if value is not None:
                a=np.array(value,dtype=float,copy=True); a.setflags(write=False); object.__setattr__(self,name,a)
        n=len(self.distance)
        if self.distance.ndim!=1 or n<2 or any(getattr(self,k).ndim!=1 or len(getattr(self,k))!=n for k in ("heading","curvature")):
            raise ValueError("track arrays must be one-dimensional, equal length, and have at least two nodes")
        if not all(np.all(np.isfinite(getattr(self,k))) for k in ("distance","heading","curvature")):
            raise ValueError("track arrays must be finite")
        if not math.isclose(float(self.distance[0]),0.,abs_tol=1e-12) or np.any(np.diff(self.distance)<=0): raise ValueError("distance must start at zero and increase")
        if (self.x_north is None)!=(self.y_east is None): raise ValueError("both XY arrays must be present or absent")
        if self.x_north is not None:
            if any(a.ndim!=1 or len(a)!=n for a in (self.x_north,self.y_east)): raise ValueError("track arrays must have equal lengths")
            if not all(np.all(np.isfinite(a)) for a in (self.x_north,self.y_east)): raise ValueError("track arrays must be finite")
        if not math.isfinite(self.maximum_radius) or self.maximum_radius<=0: raise ValueError("maximum radius must be positive and finite")

    @property
    def length(self): return float(self.distance[-1])


def load_track_data(path) -> TrackData:
    path=Path(path)
    profile=load_track(path); coords=profile.path_coordinates
    params={e.get("name"):e.get("value") for e in ET.parse(path).getroot().findall("./parameters/parameter")}
    maximum=float(params.get("maximum_radius", 300.0))
    return TrackData(profile.distance,coords.x_north if coords else None,coords.y_east if coords else None,
        coords.heading if coords else np.zeros_like(profile.distance),profile.curvature,profile.is_closed,profile.title,maximum)


def reverse_track(track: TrackData) -> TrackData:
    d=track.distance
    if track.x_north is None:
        values = track.curvature[::-1]
        return TrackData(np.r_[0.,np.cumsum(np.diff(d)[::-1])],None,None,np.zeros_like(d),values,track.is_closed,track.title,track.maximum_radius)
    if track.is_closed:
        idx=np.r_[0,np.arange(len(d)-2,0,-1),0]
        nd=np.r_[0.,np.cumsum(np.diff(d)[::-1])]
        h=np.unwrap(track.heading[idx]+np.pi)
        c=np.r_[-track.curvature[:-1][::-1], -track.curvature[-2]]
        return TrackData(nd,track.x_north[idx],track.y_east[idx],h,c,True,track.title,track.maximum_radius)
    idx=np.arange(len(d)-1,-1,-1)
    return TrackData(np.r_[0.,np.cumsum(np.diff(d)[::-1])],track.x_north[idx],track.y_east[idx],np.unwrap(track.heading[idx]+np.pi),-track.curvature[idx],False,track.title,track.maximum_radius)


def from_closed_geometry(geometry: dict, title: str = "Closed track", maximum_radius: float = 300.0) -> TrackData:
    """Convert build_closed_track geometry (north/east keys) to immutable track data."""
    distance = geometry["distance"]
    north = geometry.get("x_north", geometry.get("north"))
    east = geometry.get("y_east", geometry.get("east"))
    if north is None or east is None:
        raise ValueError("closed geometry must provide north/x_north and east/y_east arrays")
    if len(distance) and (north[0] != north[-1] or east[0] != east[-1]):
        distance = np.r_[distance, float(distance[-1]) + float(distance[-1] - distance[-2])]
        north = np.r_[north, north[0]]
        east = np.r_[east, east[0]]
        heading = np.r_[geometry["heading"], float(geometry["heading"][-1] + (geometry["heading"][1] - geometry["heading"][0]))]
        curvature = np.r_[geometry["curvature"], geometry["curvature"][0]]
    else:
        heading, curvature = geometry["heading"], geometry["curvature"]
    return TrackData(distance,north,east,heading,curvature,True,title,maximum_radius)


def from_open_course(course: OpenCourse, title: str = "Open track", maximum_radius: float = 300.0) -> TrackData:
    return TrackData(course.distance,course.x_north,course.y_east,course.heading,course.curvature,False,title,maximum_radius)


def _spline(track):
    try: from scipy.interpolate import CubicSpline
    except ImportError as exc: raise ValueError("SciPy is required to reconstruct strict open sections; install the forward extra") from exc
    return CubicSpline(track.distance,track.x_north,bc_type="periodic"),CubicSpline(track.distance,track.y_east,bc_type="periodic")


def _interp_polyline(track,s):
    # Linear interpolation with periodic wrap, used for a conservative fidelity check.
    d=track.distance; q=np.asarray(s)%track.length
    i=np.searchsorted(d,q,side="right")-1; i=np.clip(i,0,len(d)-2); f=(q-d[i])/(d[i+1]-d[i])
    return np.column_stack((track.x_north[i]+f*(track.x_north[i+1]-track.x_north[i]),track.y_east[i]+f*(track.y_east[i+1]-track.y_east[i])))


def _section_courses(track,cuts):
    if track.x_north is None or not track.is_closed: raise ValueError("section export requires signed closed XY geometry")
    L=track.length
    if not np.allclose([track.x_north[0],track.y_east[0]],[track.x_north[-1],track.y_east[-1]],atol=1e-8): raise ValueError("closed XY geometry must repeat its start point")
    raw=[float(c) for c in cuts]
    if not raw or any(not math.isfinite(c) or c<0 or c>L for c in raw): raise ValueError("cuts must be finite distances between zero and track length")
    raw=[0.0 if math.isclose(c,L,rel_tol=0,abs_tol=1e-10) else c for c in raw]
    if len(set(raw))!=len(raw): raise ValueError("cut distances must be distinct (track length and zero are the same seam)")
    if len(raw)<2: raise ValueError("at least two distinct cuts are required")
    sx,sy=_spline(track); bounds=sorted(raw); courses=[]
    for st,en0 in zip(bounds,bounds[1:]+[bounds[0]+L]):
        span=en0-st
        step=0.1
        last_error=None
        while True:
            count=max(2,int(math.ceil(span/step)))
            if count+1>MAX_SECTION_NODES: raise ValueError("section exceeds the node resource limit while refining strict-loader geometry")
            u=np.linspace(st,en0,count+1); q=u%L
            n=sx(q); e=sy(q); dn=sx(q,1); de=sy(q,1); ddn=sx(q,2); dde=sy(q,2); speed=np.hypot(dn,de)
            if np.any(speed<1e-9): raise ValueError("section spline has a stationary point")
            dist=np.r_[0.,np.cumsum(np.diff(u)*(speed[:-1]+speed[1:])*0.5)]
            h=np.unwrap(np.arctan2(de,dn)); k=(dn*dde-de*ddn)/speed**3
            h[-1]=h[-2]+((h[-1]-h[-2]+np.pi)%(2*np.pi)-np.pi)
            source=_interp_polyline(track,u)
            departure=float(np.max(np.hypot(n-source[:,0],e-source[:,1])))
            if departure>MAX_RECONSTRUCTION_DEPARTURE_M: raise ValueError(f"section reconstruction departs {departure:.6g} m, above {MAX_RECONSTRUCTION_DEPARTURE_M:.3g} m tolerance")
            try:
                from simplified_vehicle_models.tracks.profile import _validate_open_geometry_arrays
                _validate_open_geometry_arrays(dist,n,e,h,k,Path("section reconstruction"))
                break
            except ValueError as exc:
                last_error=exc
                if count+1>=MAX_SECTION_NODES: raise ValueError(f"strict-loader validation still fails at the section node limit: {last_error}") from exc
                step*=0.5
        courses.append((OpenCourse(dist,n,e,h,k,(0.,float(dist[-1])),("section",),()),departure))
    return courses


def split_closed(track,cuts):
    return [TrackData(c.distance,c.x_north,c.y_east,c.heading,c.curvature,False,track.title,track.maximum_radius) for c,_ in _section_courses(track,list(cuts))]


def _write_one(track,folder,stem,model_id,title,provenance):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    if track.x_north is not None:
        if track.is_closed:
            rows=zip(track.distance,track.x_north,track.y_east,track.heading,track.curvature)
            with (folder/f"{stem}.csv").open("w",newline="",encoding="utf8") as f:
                w=csv.writer(f); w.writerow(["distance","x_north","y_east","heading_rad","curvature"]); w.writerows(rows)
            subtype="closed-xy-profile"; closed="true"; duplicate="true"
            body="".join(f'<parameter name="{n}" value="{v}" />\n' for n,v in (("is_closed",closed),("terminal_segment_is_duplicate",duplicate),("maximum_radius",track.maximum_radius)))
            txt=f'<?xml version="1.0" encoding="UTF-8"?>\n<model schema-version="1.0" id="{escape(model_id, {"\\\"":"&quot;"})}" type="track-model" subtype="{subtype}"><metadata><title>{escape(title)}</title></metadata><data ref="{stem}.csv"/><parameters>{body}</parameters></model>\n'
            (folder/f"{stem}.xml").write_text(txt,encoding="utf8")
        else:
            course=OpenCourse(track.distance,track.x_north,track.y_east,track.heading,track.curvature,(0.,track.length),("track",),())
            (folder/f"{stem}.csv").write_text(csv_text(course),encoding="utf8")
            xml_path = folder/f"{stem}.xml"
            xml_path.write_text(xml_text(course,model_id,title,f"{stem}.csv",[provenance] if provenance else []),encoding="utf8")
            root = ET.parse(xml_path).getroot()
            if provenance:
                root.insert(1, ET.Comment(provenance.replace("--", "—")))
            maximum = root.find("./parameters/parameter[@name='maximum_radius']")
            if maximum is None:
                raise ValueError("open track XML has no maximum_radius parameter")
            maximum.set("value", repr(float(track.maximum_radius)))
            ET.ElementTree(root).write(xml_path, encoding="UTF-8", xml_declaration=True)
    else:
        # Legacy unsigned distance/radius profile. Keep radius magnitudes, never add signed geometry.
        with (folder/f"{stem}.csv").open("w",newline="",encoding="utf8") as f:
            w=csv.writer(f); w.writerow(["distance","segment_length","radius"])
            seg=np.diff(track.distance)
            rows=len(track.distance)
            for i,d in enumerate(track.distance[:rows]):
                sl=seg[i] if i < len(seg) else seg[-1]
                radius=1.0/max(abs(track.curvature[i]),1e-12)
                w.writerow((float(d),float(sl),float(radius)))
        txt=f'<?xml version="1.0" encoding="UTF-8"?>\n<model schema-version="1.0" id="{escape(model_id)}" type="track-model" subtype="distance-radius-profile"><metadata><title>{escape(title)}</title></metadata><data ref="{stem}.csv"/><parameters><parameter name="is_closed" value="{str(track.is_closed).lower()}"/><parameter name="terminal_segment_is_duplicate" value="true"/><parameter name="maximum_radius" value="{track.maximum_radius}"/></parameters></model>\n'
        (folder/f"{stem}.xml").write_text(txt,encoding="utf8")


def _save_many(tracks,directory,stems,model_id,title,*,allow_models,overwrite,overwrite_generated,provenance=None):
    directory=Path(directory); check_model_id(model_id); check_title(title)
    for stem in stems:
        check_name(stem,"file name"); _guard_targets(directory,stem,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated)
    with tempfile.TemporaryDirectory() as temp:
        scratch=Path(temp)
        for i,(t,stem) in enumerate(zip(tracks,stems)):
            _write_one(t,scratch,stem,model_id,title,(provenance[i] if provenance else ""))
            loaded=load_track(scratch/f"{stem}.xml")
            if loaded.is_closed!=t.is_closed: raise ValueError("saved track closure status changed")
        directory.mkdir(parents=True,exist_ok=True)
        backups=[]; installed=[]
        try:
            for stem in stems:
                for ext in (".csv",".xml"):
                    target=directory/f"{stem}{ext}"
                    if target.exists():
                        backup=scratch/f"backup-{len(backups)}"; os.replace(target,backup); backups.append((target,backup))
            for stem in stems:
                for ext in (".csv",".xml"):
                    target=directory/f"{stem}{ext}"; os.replace(scratch/f"{stem}{ext}",target); installed.append(target)
        except Exception:
            for p in installed:
                try:p.unlink()
                except FileNotFoundError:pass
            for target,backup in backups:
                if backup.exists():os.replace(backup,target)
            raise
    return [SavedTrack(directory/f"{s}.xml",directory/f"{s}.csv",float(load_track(directory/f"{s}.xml").lap_length),t.is_closed,len(t.distance)) for t,s in zip(tracks,stems)]


def save_track_data(track,directory,stem,model_id,title,*,allow_models=False,overwrite=False,overwrite_generated=False):
    return _save_many([track],directory,[stem],model_id,title,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated)[0]


def save_sections(track,cuts,directory,stem,model_id,title,*,allow_models=False,overwrite=False,overwrite_generated=False):
    pairs=_section_courses(track,list(cuts)); tracks=[TrackData(c.distance,c.x_north,c.y_east,c.heading,c.curvature,False,title,track.maximum_radius) for c,_ in pairs]
    stems=[f"{stem}_{i:02d}" for i in range(1,len(tracks)+1)]
    provenance=[f"Section reconstructed from periodic cubic XY spline; measured maximum departure from source polyline: {d:.9g} m; accepted limit {MAX_RECONSTRUCTION_DEPARTURE_M:g} m." for _,d in pairs]
    return _save_many(tracks,directory,stems,model_id,title,allow_models=allow_models,overwrite=overwrite,overwrite_generated=overwrite_generated,provenance=provenance)
