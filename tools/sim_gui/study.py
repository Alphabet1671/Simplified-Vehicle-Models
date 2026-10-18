"""Study definitions, sweep expansion, and persistent case indexes."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import product
import json
import os
from pathlib import Path
import subprocess
from typing import Literal, Any

from . import REPOSITORY_ROOT
from .catalogue import fits, read_entry
from .settings import RunSettings
from .vehicle import VehicleConfiguration

MAXIMUM_CASES = 500
WARNING_CASES = 50

class TooManyCases(ValueError):
    pass

@dataclass(frozen=True, slots=True)
class Dimension:
    key: str
    values: tuple[str, ...]
    labels: tuple[str, ...] = ()
    sweep: str | None = None
    points: tuple[dict, ...] = ()
    def __post_init__(self):
        values = tuple(self.values)
        labels = tuple(self.labels) or tuple(Path(v).stem for v in values)
        if len(labels) != len(values): raise ValueError("dimension labels and values differ")
        object.__setattr__(self, "values", values); object.__setattr__(self, "labels", labels)
        object.__setattr__(self, "points", tuple(self.points))

@dataclass(frozen=True, slots=True)
class CaseSpec:
    id: str
    label: str
    model: str
    track: str
    choices: dict[str, str]
    sweep_points: dict[str, dict]

@dataclass(frozen=True, slots=True)
class StudyDefinition:
    name: str
    source: str
    track: str
    model: str
    choices: dict[str, str]
    settings: RunSettings
    mode: Literal["one-at-a-time", "full-factorial"]
    dimensions: tuple[Dimension, ...]
    def __post_init__(self):
        keys = [d.key for d in self.dimensions]
        if len(keys) != len(set(keys)):
            raise ValueError(f"slot {next(k for k in keys if keys.count(k)>1)} is already swept")
    def to_json(self) -> dict[str, Any]:
        return {"name":self.name,"source":self.source,"track":self.track,"model":self.model,
                "choices":dict(self.choices),"settings":self.settings.to_json(),"mode":self.mode,
                "dimensions":[asdict(d) for d in self.dimensions]}
    @classmethod
    def from_json(cls, data):
        return cls(data["name"],data["source"],data["track"],data["model"],dict(data.get("choices",{})),
                    RunSettings.from_json(data["settings"]),data["mode"],tuple(Dimension(**d) for d in data["dimensions"]))

def _case_label(changes):
    return "baseline" if not changes else ", ".join(f"{k}={v}" for k,v in changes)

def expand(definition: StudyDefinition) -> tuple[CaseSpec, ...]:
    dims = definition.dimensions
    combinations = []
    if definition.mode == "full-factorial":
        combinations = list(product(*[range(len(d.values)) for d in dims]))
    else:
        combinations = [tuple(0 for _ in dims)]
        for n,d in enumerate(dims):
            for i in range(len(d.values)):
                if i == 0: continue
                c = [0]*len(dims); c[n] = i; combinations.append(tuple(c))
    if len(combinations) > MAXIMUM_CASES: raise TooManyCases(f"study has {len(combinations)} cases; maximum is {MAXIMUM_CASES}")
    cases=[]
    for idx, combo in enumerate(combinations):
        choices=dict(definition.choices); model=definition.model; track=definition.track; labels=[]; points={}
        for d,i in zip(dims,combo):
            value=d.values[i]
            if d.key == "model": model=value
            elif d.key == "track": track=value
            else: choices[d.key]=value
            if i != 0:
                labels.append((d.key, d.labels[i]))
                if d.sweep is not None and d.points: points[d.key]=dict(d.points[i])
        cases.append(CaseSpec(f"case_{idx:03d}", _case_label(labels), model, track, choices, points))
    return tuple(cases)

def dimension_from_sweep(folder: Path, configuration: VehicleConfiguration) -> Dimension:
    folder=Path(folder); data=json.loads((folder/"sweep.json").read_text())
    if data.get("schema") != "component-sweep-v1": raise ValueError("unsupported sweep schema")
    members=data.get("members",[])
    if not members: raise ValueError("sweep has no members")
    first=folder / members[0]["file"]
    entry=read_entry(first); matches=[k for k,p in configuration.choices.items() if fits(read_entry(p),entry)]
    if len(matches)!=1: raise ValueError("sweep members fit zero or more than one slot")
    vals=tuple((folder/m["file"]).resolve().relative_to(REPOSITORY_ROOT).as_posix() if (folder/m["file"]).resolve().is_relative_to(REPOSITORY_ROOT) else str((folder/m["file"]).resolve()) for m in members)
    points=tuple({"sweep": str(folder), "index":m["index"], "target":data.get("target",""), "operation":data.get("operation",""), "x":m["x"], "unit":data.get("unit","")} for m in members)
    return Dimension(matches[0],vals,tuple(m["label"] for m in members),str(folder),points)

def _atomic(path, data):
    tmp=path.with_name(path.name+".tmp"); tmp.write_text(json.dumps(data,indent=2,sort_keys=True,allow_nan=False)); os.replace(tmp,path)

class StudyIndex:
    def __init__(self,directory, payload): self.directory=Path(directory); self._payload=payload
    @property
    def cases(self): return tuple(self._payload["cases"])
    @classmethod
    def create(cls,directory,definition,cases):
        directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
        try: commit=subprocess.run(["git","rev-parse","HEAD"],capture_output=True,text=True,cwd=REPOSITORY_ROOT,check=True).stdout.strip()
        except Exception: commit=None
        payload={"schema":"sim-study-v1","name":definition.name,"created_utc":"","repository_commit":commit,"definition":"definition.study.json","cases":[asdict(c) for c in cases]}
        _atomic(directory/"definition.study.json",definition.to_json()); _atomic(directory/"study.json",payload); return cls(directory,payload)
    @classmethod
    def load(cls,directory): return cls(directory,json.loads((Path(directory)/"study.json").read_text()))
    def update(self,case_id,**fields):
        for case in self._payload["cases"]:
            if case["id"]==case_id: case.update(fields); break
        else: raise KeyError(case_id)
        _atomic(self.directory/"study.json",self._payload)
