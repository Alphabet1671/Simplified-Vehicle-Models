"""Strict, immutable-source CSV playback with canonical SI commands."""
from dataclasses import dataclass
import csv
import hashlib
import math
from pathlib import Path
from .schema import convert_unit
from .types import RobotCommand

_MAX_BYTES=100*1024*1024
_MAX_ROWS=1_000_000

@dataclass(frozen=True, slots=True)
class CsvMapping:
    time: str|None
    swa: str
    accelerator: str
    brake: str
    time_unit: str
    swa_unit: str
    pedal_unit: str
    interpolation: str
    sample_dt_s: float|None=None
    rebase: bool=False
    delimiter: str=','
    decimal: str='.'

    def __post_init__(self):
        if not all(isinstance(x,str) and x for x in (self.swa,self.accelerator,self.brake)):raise ValueError('all three input columns must be mapped')
        if self.time is not None and (not isinstance(self.time,str) or not self.time):raise ValueError('time column mapping invalid')
        if self.time_unit not in {'s','ms'} or self.swa_unit not in {'deg','rad'} or self.pedal_unit not in {'percent','fraction'}:raise ValueError('explicit supported units are required')
        if self.interpolation not in {'hold','linear'}:raise ValueError('interpolation must be hold or linear')
        if self.delimiter not in {',',';'} or self.decimal not in {'.',','}:raise ValueError('unsupported delimiter/decimal')
        if self.delimiter==self.decimal:raise ValueError('delimiter and decimal convention conflict')
        if self.time is None:
            if isinstance(self.sample_dt_s,bool) or not isinstance(self.sample_dt_s,(int,float)) or not math.isfinite(self.sample_dt_s) or self.sample_dt_s<=0:raise ValueError('three-column input requires positive sample_dt_s')
        elif self.sample_dt_s is not None:raise ValueError('sample_dt_s only applies when time column is absent')
        if not isinstance(self.rebase,bool):raise ValueError('rebase must be boolean')

@dataclass(frozen=True, slots=True)
class CsvSource:
    time_s: tuple[float,...]
    commands: tuple[RobotCommand,...]
    source_sha256: str
    mapping: CsvMapping
    source_bytes: bytes

    def breakpoints(self):return self.time_s
    def sample(self,time_s):
        if isinstance(time_s,bool) or not isinstance(time_s,(int,float)) or not math.isfinite(time_s):raise ValueError('request time must be finite')
        if time_s<0 or time_s>self.time_s[-1]:raise ValueError('request outside CSV source bounds')
        import bisect
        i=bisect.bisect_right(self.time_s,time_s)-1
        if self.mapping.interpolation=='hold' or i==len(self.time_s)-1:return self.commands[i]
        alpha=(time_s-self.time_s[i])/(self.time_s[i+1]-self.time_s[i]); a,b=self.commands[i:i+2]
        return RobotCommand(*(x+(y-x)*alpha for x,y in zip((a.swa_rad,a.accelerator,a.brake),(b.swa_rad,b.accelerator,b.brake))))

def load_csv_source(path:Path,mapping:CsvMapping)->CsvSource:
    if not isinstance(mapping,CsvMapping):raise TypeError('mapping must be CsvMapping')
    raw=Path(path).read_bytes()
    if len(raw)>_MAX_BYTES:raise ValueError('CSV exceeds 100 MiB limit')
    try:text=raw.decode('utf-8-sig')
    except UnicodeDecodeError as exc:raise ValueError('CSV must be UTF-8') from exc
    try: rows=list(csv.reader(text.splitlines(),delimiter=mapping.delimiter,strict=True))
    except csv.Error as exc:raise ValueError(f'malformed CSV: {exc}') from exc
    while rows and not any(cell.strip() for cell in rows[-1]):rows.pop()
    if not rows:raise ValueError('CSV is empty')
    header=[x.strip() for x in rows[0]]
    if len(set(header))!=len(header) or any(not x for x in header):raise ValueError('CSV header has empty or duplicate columns')
    names=[mapping.swa,mapping.accelerator,mapping.brake]+([mapping.time] if mapping.time else [])
    if any(x not in header for x in names):raise ValueError('unmapped input column')
    indexes=[header.index(x) for x in names]; si,ai,bi=indexes[:3]; ti=indexes[3] if mapping.time else None
    data=[]
    for rownum,row in enumerate(rows[1:],2):
        if not any(x.strip() for x in row):continue
        if rownum>_MAX_ROWS+1:raise ValueError('CSV exceeds 1,000,000 data rows')
        if len(row)!=len(header):raise ValueError(f'row {rownum}: malformed column count')
        def number(j,label):
            token=row[j].strip()
            if not token:raise ValueError(f'row {rownum}, column {header[j]}: empty value')
            if mapping.decimal==',':token=token.replace(',','.')
            try:v=float(token)
            except ValueError as exc:raise ValueError(f'row {rownum}, column {header[j]}: invalid number') from exc
            if not math.isfinite(v):raise ValueError(f'row {rownum}, column {header[j]}: nonfinite value')
            return v
        raw_s,raw_a,raw_b=(number(j,header[j]) for j in (si,ai,bi))
        t=number(ti,header[ti]) if ti is not None else len(data)*mapping.sample_dt_s
        t=convert_unit(t,mapping.time_unit,'s')
        swa=convert_unit(raw_s,mapping.swa_unit,'rad'); acc=convert_unit(raw_a,mapping.pedal_unit,'fraction');brake=convert_unit(raw_b,mapping.pedal_unit,'fraction')
        if abs(swa)>math.pi:raise ValueError(f'row {rownum}, column {header[si]}: steering outside +/- pi rad envelope')
        if not 0<=acc<=1 or not 0<=brake<=1:raise ValueError(f'row {rownum}: pedal outside [0,1]')
        if acc>0 and brake>0:raise ValueError(f'row {rownum}: pedal overlap')
        data.append((t,RobotCommand(swa,acc,brake)))
    if len(data)<2:raise ValueError('CSV requires at least two rows')
    ts=[x[0] for x in data]
    if any(t<0 for t in ts):raise ValueError('time must be nonnegative')
    if any(b<=a for a,b in zip(ts,ts[1:])):raise ValueError('time must be strictly increasing without duplicates')
    if ts[0]!=0:
        if not mapping.rebase:raise ValueError('nonzero time origin requires explicit rebase')
        origin=ts[0];ts=[x-origin for x in ts]
    if mapping.interpolation=='linear':
        for i,(a,b) in enumerate(zip(data,data[1:])):
            if (a[1].accelerator>0 and b[1].brake>0) or (a[1].brake>0 and b[1].accelerator>0):raise ValueError(f'linear pedal crossing between rows {i+2} and {i+3}')
    return CsvSource(tuple(ts),tuple(x[1] for x in data),hashlib.sha256(raw).hexdigest(),mapping,raw)
