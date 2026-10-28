"""Generate the fixed-replay IEM24 MIS measured-versus-simulated report.

This consumer never evaluates vehicle physics.  It reads the saved forward replay,
resamples its accepted-step channels to 100 Hz, applies the report filter, and
compares that one trace with the already-conditioned measured stint.
"""
from __future__ import annotations
import argparse, csv, hashlib, json, os, bisect
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

if __package__ in (None, ""):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simplified_vehicle_models.results.run_record import load_run_record
from simplified_vehicle_models.planning.yaw_lap_artifact import ARRAYS, _digest
from simplified_vehicle_models.tracks.profile import load_track
from tools.data.iem24_telemetry import zero_phase_lowpass
from tools.reports.compare_iem24_murphy_stint import (load_table, flying_lap_indices, fit_position_offset, project_onto_track, fold_laps, unwrap_track_distance)

CHANNELS=(
 ("speed_mps","Speed","m/s"),
 ("longitudinal_acceleration_mps2","Longitudinal acceleration","m/s²"),
 ("lateral_acceleration_mps2","Lateral acceleration","m/s²"),
 ("yaw_rate_rad_s","Yaw rate","rad/s"),
 ("motor_torque_request_nm","Motor torque request (per-motor reconstructed)","N·m"),
 ("pack_voltage_v","Pack terminal voltage","V"),)
BINS=500; LIMIT=10.5; CUTOFF=1.; DT=.01
ROOT=Path(__file__).resolve().parents[2]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()
def metric(a,b):
 e=a-b; sd=np.std(b)
 return {"rmse":float(np.sqrt(np.mean(e*e))),"mae":float(np.mean(abs(e))),"bias":float(np.mean(e)),"correlation":float(np.corrcoef(a,b)[0,1]),"normalised_rmse":float(np.sqrt(np.mean(e*e))/sd) if sd else None}
def cumulative_time_on_reference(t, reference_s, physical_s, reference_grid, *, lap_length=None, boundary_tolerance=1.0):
  """Interpolate elapsed time only when full-lap station endpoints are evidenced.

  Reference station is sampled through the plan Jacobian. Physical station is
  retained for monotonicity validation but is never substituted for reference.
  """
  t=np.asarray(t,float); rs=np.asarray(reference_s,float); ps=np.asarray(physical_s,float); grid=np.asarray(reference_grid,float)
  if t.ndim!=1 or len(t)<2 or len(t)!=len(rs) or len(t)!=len(ps): raise ValueError('time and mapped stations must have equal length')
  if not np.all(np.isfinite(t)) or not np.all(np.isfinite(rs)) or not np.all(np.isfinite(ps)) or np.any(np.diff(t)<=0): raise ValueError('time/station samples must be finite with strictly increasing time')
  if np.any(np.diff(rs)<-1e-8) or np.any(np.diff(ps)<-1e-8): raise ValueError('physical/reference station mappings must be monotonic')
  if lap_length is None: lap_length=float(grid[-1]) if grid.size else None
  if lap_length is None or not np.isfinite(lap_length) or lap_length<=0: raise ValueError('full-lap length is required')
  rs=rs-rs[0];ps=ps-ps[0]
  if rs[0]>boundary_tolerance or ps[0]>boundary_tolerance or rs[-1]<lap_length-boundary_tolerance or ps[-1]<lap_length-boundary_tolerance: raise ValueError('full-lap boundary mapping not established within tolerance')
  origin=float(t[0])
  if abs(rs[-1]-lap_length)>boundary_tolerance or abs(ps[-1]-lap_length)>boundary_tolerance: raise ValueError('physical/reference lap periods disagree with requested length')
  reverse_unique=np.unique(rs[::-1],return_index=True)[1];keep=np.sort(len(rs)-1-reverse_unique);rs=rs[keep];ps=ps[keep];t=t[keep]-origin
  rs[0]=0.;ps[0]=0.;rs[-1]=lap_length
  if np.any(np.diff(rs)<=0): raise ValueError('reference station map is not strictly monotone after duplicate handling')
  if np.any(np.diff(ps)<-1e-8): raise ValueError('physical station map reverses after duplicate handling')
  if np.any(grid<0) or np.any(grid>lap_length): raise ValueError('requested interpolation station lies outside evidenced lap')
  return np.interp(grid,rs,t)

def prepare_full_lap(time, station, lap_length, *, start_time=None, boundary_duration=None, boundary_tolerance=1.0):
  """Derive full-lap endpoints locally from a monotone, already-unwrapped lap."""
  t=np.asarray(time,float); raw_s=np.asarray(station,float); L=float(lap_length)
  if t.ndim!=1 or raw_s.ndim!=1 or len(t)<2 or len(t)!=len(raw_s): raise ValueError('lap time/station arrays must be equal-length vectors')
  if not np.all(np.isfinite(t)) or not np.all(np.isfinite(raw_s)) or np.any(np.diff(t)<=0) or np.any(np.diff(raw_s)<-1e-8): raise ValueError('lap samples must be finite, time-increasing, and station-monotone')
  start=float(t[0] if start_time is None else start_time); s=raw_s-raw_s[0]
  if s[-1]<L-boundary_tolerance or s[-1]>L+boundary_tolerance: raise ValueError('full-lap traversal endpoints unsupported by measured samples')
  if boundary_duration is not None and (not np.isfinite(boundary_duration) or boundary_duration<=0): raise ValueError('declared lap boundary duration must be finite and positive')
  def local_time(target):
   j=int(np.searchsorted(s,target,side='left'))
   if j<len(s) and abs(s[j]-target)<1e-9: return float(t[j]-start)
   if j==0:
    if s[0]-target>boundary_tolerance or s[1]<=s[0]: raise ValueError('full-lap start boundary cannot be derived locally')
    f=(target-s[0])/(s[1]-s[0]); return max(0.,float(t[0]+f*(t[1]-t[0])-start))
   if j==len(s):
    if target-s[-1]>boundary_tolerance or s[-1]<=s[-2]: raise ValueError('full-lap end boundary cannot be derived locally')
    f=(target-s[-2])/(s[-1]-s[-2]); return float(t[-2]+f*(t[-1]-t[-2])-start)
   f=(target-s[j-1])/(s[j]-s[j-1]); return float(t[j-1]+f*(t[j]-t[j-1])-start)
  start_elapsed=local_time(0.)
  end_elapsed=float(boundary_duration) if boundary_duration is not None else local_time(L)
  keep=np.r_[s[1:]!=s[:-1],True]; s=s[keep]; elapsed=t[keep]-start
  if np.any(np.diff(s)<=0): raise ValueError('duplicate station samples cannot define strict full-lap map')
  if s[0]>boundary_tolerance or s[-1]<L-boundary_tolerance: raise ValueError('full-lap seam samples do not support exact endpoints')
  s[0]=0.;s[-1]=L;elapsed[0]=start_elapsed;elapsed[-1]=end_elapsed
  return elapsed,s

def map_lap_through_plan(physical_s, plan_physical, plan_reference, lap_length, *, tolerance=1.0):
  """Apply the saved path Jacobian; extend exact endpoints only when periods agree."""
  p=np.asarray(physical_s,float); xp=np.asarray(plan_physical,float); yr=np.asarray(plan_reference,float); L=float(lap_length)
  if len(xp)<2 or len(xp)!=len(yr) or np.any(np.diff(xp)<=0) or np.any(np.diff(yr)<=0): raise ValueError('saved physical-to-reference mapping is not strictly monotone')
  period=float(xp[-1]-xp[0])
  if abs(period-L)>tolerance or abs(float(yr[-1]-yr[0])-L)>tolerance: raise ValueError('saved Jacobian does not establish requested lap period')
  if p[0]>tolerance or p[-1]<period-tolerance or p[-1]>period+tolerance: raise ValueError('physical lap endpoints unsupported by saved Jacobian')
  mapped=np.interp(p,xp-xp[0],yr-yr[0])
  mapped[0]=0.;mapped[-1]=L
  return mapped

def sector_time_gaps(t, reference_s, physical_s, sector=50.):
  """Return 50 m reference sectors plus remainder using the declared path map."""
  # Inputs are time samples and physical s; convert their mapped reference station.
  t=np.asarray(t,float); rs=np.asarray(reference_s,float); ps=np.asarray(physical_s,float)
  if len(t)!=len(rs) or len(rs)!=len(ps): return []
  if np.any(np.diff(rs)<0) or np.any(np.diff(ps)<0): raise ValueError('sector station mapping must be monotonic')
  if np.any(np.diff(rs)==0):
   unique=np.unique(rs); indexes=np.array([np.flatnonzero(rs==s)[-1] for s in unique]); rs=rs[indexes]; ps=ps[indexes]; t=t[indexes]
  L=float(rs[-1]); edges=list(np.arange(0,L,sector))+[L]; rows=[]
  for a,b in zip(edges[:-1],edges[1:]):
   ia=np.searchsorted(rs,a,side='left'); ib=np.searchsorted(rs,b,side='right')-1
   if ib<=ia: continue
   rows.append({'start_m':float(a),'end_m':float(b),'duration_s':float(t[ib]-t[ia]),'physical_distance_m':float(ps[ib]-ps[ia])})
  return rows

def paired_sector_gap_table(sim_time,sim_reference,meas_time,meas_reference,L,sector=50.,*,sim_physical=None,meas_physical=None):
  """Align full-lap elapsed-time traces after explicit boundary validation."""
  sim_reference=np.asarray(sim_reference,float); meas_reference=np.asarray(meas_reference,float)
  sim_physical=sim_reference if sim_physical is None else np.asarray(sim_physical,float)
  meas_physical=meas_reference if meas_physical is None else np.asarray(meas_physical,float)
  edges=list(np.arange(0.,L,sector))+[float(L)]
  sim_cumulative=cumulative_time_on_reference(sim_time,sim_reference,sim_physical,edges,lap_length=L)
  meas_cumulative=cumulative_time_on_reference(meas_time,meas_reference,meas_physical,edges,lap_length=L)
  return [{'start_m':float(a),'end_m':float(b),'simulated_s':float(sim_cumulative[i+1]-sim_cumulative[i]),'measured_s':float(meas_cumulative[i+1]-meas_cumulative[i]),'signed_gap_sim_minus_meas_s':float((sim_cumulative[i+1]-sim_cumulative[i])-(meas_cumulative[i+1]-meas_cumulative[i]))} for i,(a,b) in enumerate(zip(edges[:-1],edges[1:]))]

def _not_available(reason): return {'status':'unavailable','reason':reason}
def interp_periodic(s,v,g,L):
 o=np.argsort(s); s=s[o];v=v[o]
 return np.interp(g,np.r_[s-L,s,s+L],np.tile(v,3))
REQUIRED_CHANNELS={"speed", "acceleration-longitudinal", "acceleration-lateral", "throttle", "bus-voltage", "soc", "wheel-drive-torque"}
REQUIRED_STATES={"s", "yaw_rate"}

def validate_record_plan(record, plan_metadata, *, require_complete=True):
  """Structural admission checks only; this reporting path never runs physics."""
  t=np.asarray(record.time, dtype=float)
  if t.ndim != 1 or t.size < 2 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
   raise ValueError("record time must be finite and strictly increasing")
  if require_complete and record.manifest.segment.get("lap_completed") is not True:
   raise ValueError("record does not declare lap_completed=true; refusing complete-lap claim")
  missing_channels=REQUIRED_CHANNELS-set(record.manifest.channel_names)
  missing_states=REQUIRED_STATES-set(record.manifest.state_names)
  if not (len(record.manifest.channel_units)==len(record.manifest.channel_names)==len(record.manifest.channel_frames)):
   raise ValueError("channel unit/frame provenance declarations are incomplete")
  if not record.manifest.sources:
   # Preserve historical unknown source hashes as unknown; do not invent them.
   pass
  if missing_channels or missing_states:
   raise ValueError(f"record channel/state declarations incomplete: channels={sorted(missing_channels)}, states={sorted(missing_states)}")
  source=record.manifest.model.get("yaw_profile",{})
  for key in ("array_digest","track_identity","map_identity"):
   if not source.get(key) or not plan_metadata.get(key) or source[key] != plan_metadata[key]:
    raise ValueError(f"record/plan {key} identity missing or mismatched")
  if not record.manifest.model.get("controller"):
   raise ValueError("record provenance is incomplete (controller identity missing)")
  # Older records bind the exact plan arrays and track/map identities but omit
  # solver/settings fields from the record manifest; retain that as unknown.
  for name in REQUIRED_CHANNELS:
   values=np.asarray(record.channel_values(name),dtype=float)
   if values.shape[0]!=t.size or not np.all(record.channel_validity(name)) or not np.all(np.isfinite(values)):
    raise ValueError(f"record channel {name!r} has invalid/missing values")
  return True

def reconstruct_torque_request(throttle, limit=LIMIT):
  throttle=np.asarray(throttle,float)
  if not np.isfinite(limit) or limit<=0 or not np.all(np.isfinite(throttle)): raise ValueError('torque limit and throttle must be finite; torque limit positive')
  return throttle*float(limit)

def record_profile(path, plan_path, grid, L, *, torque_limit=LIMIT):
 r=load_run_record(path,verify_sources=False); t=np.asarray(r.time); tu=np.arange(t[0],t[-1]+DT*.25,DT)
 def chan(n): return np.interp(tu,t,np.asarray(r.channel_values(n),float))
 # accepted steps are irregular: resample first, then identical 1 Hz first-order zero-phase filter.
 d=np.interp(tu,t,r.state_column('s'))
 with np.load(plan_path,allow_pickle=False) as plan:
  if 'metadata' not in plan or any(key not in plan for key in ARRAYS): raise ValueError(f'{plan_path}: incomplete yaw lap artifact')
  metadata=json.loads(str(plan['metadata'].item()))
  arrays={key:np.array(plan[key],float,copy=True) for key in ARRAYS}
 if metadata.get('array_digest') != _digest(arrays): raise ValueError(f'{plan_path}: yaw plan array digest mismatch')
 validate_record_plan(r,metadata)
 profile_metadata=r.manifest.model.get('yaw_profile',{})
 if profile_metadata.get('array_digest') != metadata['array_digest']: raise ValueError(f'{plan_path}: does not match replay yaw-profile digest')
 if profile_metadata.get('track_identity') != metadata.get('track_identity'): raise ValueError(f'{plan_path}: does not match replay track identity')
 if profile_metadata.get('map_identity') != metadata.get('map_identity'): raise ValueError(f'{plan_path}: does not match replay map identity')
 physical=arrays['distance']; reference=arrays['reference_distance']
 if physical.size != reference.size or physical.size < 2 or np.any(np.diff(physical)<=0) or np.any(np.diff(reference)<=0): raise ValueError(f'{plan_path}: invalid station map')
 if d[0] < physical[0]-1e-6 or d[-1] > physical[-1]+1e-6: raise ValueError('replay physical station exceeds saved plan mapping')
 # Saved plan maps plant physical station to the immutable signed-centreline station.
 # This is a source mapping, not a fitted rescaling of the replay.
 reference_d=np.interp(d,physical,reference)
 raw={"speed_mps":chan('speed'),"longitudinal_acceleration_mps2":chan('acceleration-longitudinal'),"lateral_acceleration_mps2":chan('acceleration-lateral'),"yaw_rate_rad_s":np.interp(tu,t,r.state_column('yaw_rate')),"motor_torque_request_nm":reconstruct_torque_request(chan('throttle'),torque_limit),"delivered_wheel_torque_nm":np.mean(np.column_stack([np.interp(tu,t,np.asarray(r.channel_values('wheel-drive-torque'),float)[:,i]) for i in (2,3)]),axis=1),"bus_power_w":chan('bus-power') if r.has_channel('bus-power') else None,"brake_force_n":chan('brake-force') if r.has_channel('brake-force') else None,"brake_fraction":chan('brake-fraction') if r.has_channel('brake-fraction') else None,"steering_rate_rad_s":chan('steering-rate') if r.has_channel('steering-rate') else None,"pack_voltage_v":chan('bus-voltage'),"soc":chan('soc'),"throttle":chan('throttle')}
 accepted={"speed_mps":np.asarray(r.channel_values('speed'),float),"pack_voltage_v":np.asarray(r.channel_values('bus-voltage'),float),"soc":np.asarray(r.channel_values('soc'),float)}
 filtered={k:zero_phase_lowpass(v,DT,CUTOFF,order=1) if v is not None else None for k,v in raw.items()}
 profile={k:interp_periodic(np.mod(reference_d,L),v,grid,L) for k,v in filtered.items() if v is not None and k not in ('soc','throttle')}
 return r,tu,d,reference_d,raw,accepted,filtered,profile,float(physical[-1]),float(reference[-1]),physical,reference
def fig(out,g,L,folded,med,sim,tu,d,filtered,raw,accepted,accepted_time):
 os.environ.setdefault('MPLCONFIGDIR','/tmp/matplotlib-config'); import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as p
 f,axs=p.subplots(6,1,figsize=(12,16.5),sharex=True)
 for ax,(k,n,u) in zip(axs,CHANNELS):
  x=folded[k];ax.fill_between(g,x.min(0),x.max(0),color='#9dc3e6',alpha=.55,label='Measured range (10 flying laps)');ax.plot(g,med[k],color='#1f4e79',lw=1.7,label='Measured median');ax.plot(g,sim[k],color='#c00000',lw=1.7,label='Saved replay');ax.set_ylabel(f'{n} ({u})');ax.grid(alpha=.25);ax.legend(fontsize=7,ncol=3,loc='upper right')
 axs[-1].set_xlabel('MIS centreline distance (m)');f.suptitle('IEM24 Murphy — fixed yaw-aware replay vs measured flying laps');f.tight_layout();f.savefig(out/'comparison.png',dpi=150);p.close(f)
 f,ax=p.subplots(2,1,figsize=(12,7));
 for row in folded['speed_mps']:ax[0].plot(g,row,color='#9dc3e6',lw=.8)
 ax[0].plot(g,med['speed_mps'],color='#1f4e79',lw=2,label='Measured median');ax[0].plot(g,sim['speed_mps'],color='#c00000',lw=1.7,label='Saved replay');ax[0].set(ylabel='Speed (m/s)',xlabel='MIS centreline distance (m)',title='All measured flying laps, median, and fixed replay');ax[0].legend();ax[0].grid(alpha=.25)
 ax[1].hist(folded['motor_torque_request_nm'].ravel(),bins=35,color='#1f4e79',alpha=.75,label='Measured requests');ax[1].axvline(np.max(sim['motor_torque_request_nm']),color='#c00000',label='Replay reconstructed request');ax[1].set(xlabel='Per-motor request (N·m)',ylabel='Samples',title='Second signal distribution: torque requests');ax[1].legend();ax[1].grid(alpha=.25);f.tight_layout();f.savefig(out/'lap_spread.png',dpi=150);p.close(f)
 f,axs=p.subplots(3,1,figsize=(12,8),sharex=True)
 for ax,k,n,u in zip(axs,('speed_mps','pack_voltage_v','soc'),('Speed','Pack terminal voltage','State of charge'),('m/s','V','–')):ax.plot(accepted_time,accepted[k],color='#c00000');ax.set_ylabel(f'{n} ({u})');ax.grid(alpha=.25)
 axs[-1].set_xlabel('Accepted replay time (s)');f.suptitle('Raw saved replay, including battery initial-condition channel');f.tight_layout();f.savefig(out/'simulated_lap.png',dpi=150);p.close(f)
def report(out,summary,extract,L,torque_limit=LIMIT):
 laps=extract['laps']; rows='\n'.join(f"| {q['index']}{' (standing start; excluded)' if q['standing_start'] else ' (flying)'} | {q['start_s']:.2f} | {q['lap_time_s']:.2f} | {q['distance_m']:.1f} | {q['mean_speed_mps']:.2f} | {q['closure_radius_m']:.2f} |" for q in laps)
 mr='\n'.join(f"| {n} ({u}) | {summary['metrics'][k]['rmse']:.3f} | {summary['metrics'][k]['mae']:.3f} | {summary['metrics'][k]['bias']:+.3f} | {summary['metrics'][k]['correlation']:.3f} | {summary['metrics'][k]['normalised_rmse']:.3f} |" for k,n,u in CHANNELS)
 m=summary['metrics']; sim=summary['simulation']; fit=summary['position_fit']; dist=summary['measured']['median_distance_m']; gap=sim['lap_time_s']-summary['measured']['median_lap_time_s']
 sector=summary.get('sector_comparison',{}); sector_report=(f"Sector comparison unavailable: {sector.get('reason','not computed')}" if sector.get('status')!='available' else '\n'.join(f"{x['start_m']:.0f}–{x['end_m']:.0f} m: replay {x['simulated_s']:.3f} s, measured {x['measured_s']:.3f} s, signed gap {x['signed_gap_sim_minus_meas_s']:+.3f} s" for x in sector['rows'])+'\nLargest positive: '+', '.join(f"{x['start_m']:.0f}–{x['end_m']:.0f} m ({x['signed_gap_sim_minus_meas_s']:+.3f} s)" for x in sector['largest_positive'])+'; largest negative: '+', '.join(f"{x['start_m']:.0f}–{x['end_m']:.0f} m ({x['signed_gap_sim_minus_meas_s']:+.3f} s)" for x in sector['largest_negative']))
 operational_summary='\n'.join(f"- {name}: {entry['status']} ({entry.get('unit','')})"+(f" — {entry['reason']}" if entry['status']=='unavailable' else '') for name,entry in summary.get('operational_traces',{}).items())
 text=f'''# IEM24 Murphy yaw-aware replay: simulated MIS 2024 lap vs measured endurance stint

## Configuration

- Mass: 287 kg (209 kg car + 78 kg driver); front mass fraction 45 %. CGx is 55 % of the 1.535 m wheelbase behind the front axle (the model's 45 % front static mass fraction); CG height 0.300 m; track width 1.240 m.
- Aero: CL·A = 3.5 m², CD·A = 1.55 m², aero balance 50 % at 1.22 kg/m³.
- Chassis/control: planar 7-DOF `models/formulations/iem24_murphy_real_planar_7dof.xml`; LLTD 42 %, two rear motors, 10.5 N·m controller maximum per motor, 10.9:1 gearing, 95 % drivetrain efficiency, 20 kW per motor and a current compiled motor limit of 1947.787 rad/s (18,600 rpm); mechanical brakes and no reported regeneration. Control limits include 0.5236 rad steering angle, π rad/s steering rate, 1176.8 N maximum pedal force, 0.08 s steering lag, 0.18 s pedal travel / 0.03 s pedal lag, and fixed controller gains preview 0.10, heading 1.8, lateral 2.0, yaw 0.12. The run-record manifest does not embed a current configuration source hash; this is disclosed rather than inferred.
- Initial conditions: recorded initial SOC = {sim['initial_soc']:.6f}; recorded initial terminal bus voltage = {sim['initial_voltage_v']:.3f} V. Recorded replay initial state is speed components v_x/v_y = 10.699/−0.915 m/s, yaw rate = −1.324 rad/s, heading error = 0.0853 rad and steering = −0.1877 rad; wheel-speed states are plant states. These are nominal plant initial conditions, not fitted to measured telemetry; only the stint-start battery voltage was carried into the compiled battery initialisation.
- Simulated flying lap: exactly {sim['lap_time_s']:.9f} s over {sim['physical_length_m']:.3f} m of model path arclength, mapped onto the {sim['reference_length_m']:.3f} m reference centreline on `models/tracks/mis_ev_endurance_2024_width.xml`, from the saved model-aware forward replay used for the video.

## Measured data

The conditioned cache is `docs/validation/iem24-murphy/measured_stint.csv` (19.9804878 Hz export; original shared log approximately 99.9 Hz). Its extraction metadata is `docs/validation/iem24-murphy/telemetry_extraction.json`; raw source `~/Desktop/0615CompEndurance.mat` is {summary['measured']['raw_bytes']} bytes, SHA-256 `{summary['measured']['raw_sha256']}`. The CSV samples actually consumed span 105.00–913.03 s (the eleven exported laps); extraction's broader driving window is 117.02–925.91 s and segmentation window is 105.00–938.00 s. Channels consumed are body-frame speed, `AccelX`, `AccelY`, `GyroZ`, mean(`Tq_RL_Req`,`Tq_RR_Req`), and `VPack` as documented by the extraction metadata. The cached measured channels were already yaw-rotated and low-pass filtered at 1 Hz with a zero-phase first-order filter; they are **not filtered again**. This applies a rotation/correction to raw navigation data, not a new correction here.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---:|---:|---:|---:|---:|
{rows}

All 11 measured laps are listed; lap 0 is the standing start and is flagged/excluded from flying statistics. The ten flying-lap durations have a displayed median of {summary['measured']['median_lap_time_s']:.3f} s (median of lap times, not an integral of a pointwise-median speed profile). Median measured flying lap: {summary['measured']['median_lap_time_s']:.3f} s over {dist:.1f} m, {100*(dist/L-1):+.2f} % from the {L:.1f} m modelled centreline.

### One physical lap axis and conditioning

Measured positions are fitted by a constant translation and projected/folded onto the signed closed centreline; no inferred phase shift or length scaling was applied. Fit offset is {fit['north']:+.2f} m north, {fit['east']:+.2f} m east; flying-lap centreline residual median/P95 are {fit['median']:.2f}/{fit['p95']:.2f} m. The saved replay has irregular accepted times; each six-channel trace is resampled to uniform 100 Hz, then filtered with the same 1 Hz first-order zero-phase filter before distance interpolation. Its saved plan maps physical station ({sim['physical_length_m']:.3f} m) to immutable reference centreline station ({sim['reference_length_m']:.3f} m); cumulative time/sectors honor this station Jacobian and do not treat `1/V` as `dt/ds_reference`; that fixed map, rather than fitted length rescaling, puts replay and measured profiles on the common axis. Raw replay values remain in the record. The navigation yaw-skew correction was 8.550° (IQR 0.304°); extraction reports corrected longitudinal acceleration/velocity-rate correlation 0.984 and RMSE 0.322 m/s², from 0.816 and 1.362 m/s² before the correction.

## Trace comparison

Cumulative per-lap times and sector comparison use monotone unwrapped physical traversals mapped through the saved physical-to-reference Jacobian. Measured lap-boundary elapsed time is preserved; fixed 50 m sectors include the final remainder. `1/V` is not treated as `dt/ds_reference`.

{sector_report}

Operational trace availability (full samples: `operational_traces.csv`; unavailable values are left blank):

{operational_summary}

The reference is the pointwise median of ten folded flying laps on 500 centreline-distance bins. Bias is `sim − meas`; all units are SI. Body axes are SAE: x forward, y right, z down; positive yaw rate is a right turn.

| Signal | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |
|---|---:|---:|---:|---:|---:|
{mr}

![Measured flying-lap range and median compared with the one saved replay for all six channels](comparison.png)

![Every measured flying-lap speed with median and replay, plus torque-request distribution](lap_spread.png)

![Raw saved simulated lap, including terminal voltage and initial-condition state of charge](simulated_lap.png)

## Reading the comparison

- The replay is **{abs(gap):.3f} s {'faster' if gap < 0 else 'slower'}** ({100*(sim['lap_time_s']/summary['measured']['median_lap_time_s']-1):+.2f} % relative to the measured median): {sim['lap_time_s']:.3f} s against {summary['measured']['median_lap_time_s']:.3f} s. Its mean centreline speed follows from time over the modelled centreline; this descriptive relationship is not a causal diagnosis. This is one strict feasible forward replay, not an optimum; the comparison does not establish why it differs.
- Speed RMSE is {m['speed_mps']['rmse']:.3f} m/s (bias {m['speed_mps']['bias']:+.3f} m/s; correlation {m['speed_mps']['correlation']:.3f}). Longitudinal/lateral acceleration RMSE is {m['longitudinal_acceleration_mps2']['rmse']:.3f}/{m['lateral_acceleration_mps2']['rmse']:.3f} m/s²; yaw-rate RMSE is {m['yaw_rate_rad_s']['rmse']:.3f} rad/s.
- Reconstructed per-motor torque request RMSE is {m['motor_torque_request_nm']['rmse']:.3f} N·m. It is exactly `realized throttle × {torque_limit:g} N·m` for the reporting controller limit: reconstructed request before electrical/speed limits, distinct from delivered motor torque and actual signed `wheel-drive-torque` (gear ratio/efficiency convention). Changing this reporting limit changes only reconstructed request; delivered torque stays as recorded. Electrical power limits, SOC/voltage and saturation are not collapsed into that request. The manifest's `wheel-drive-torque` is actual wheel torque and is deliberately not relabelled as request; this distinction and any power-limit effect limit interpretation.
- Pack terminal-voltage RMSE is {m['pack_voltage_v']['rmse']:.3f} V with bias {m['pack_voltage_v']['bias']:+.3f} V. The replay starts from stint-start SOC/voltage while flying laps span discharge, so this contains an initial-condition offset as well as trace differences.
- The {gap:+.3f} s gap ({100*gap/summary['measured']['median_lap_time_s']:+.2f} %) is historical context, not causal attribution: retained provenance does not identity-match historical plans/records or Task9 pairs, so all five controlled treatments are not run/inconclusive. The displayed median is the median of {summary['measured']['flying_laps_used']} flying-lap durations ({summary['measured']['median_lap_time_s']:.3f} s), not the integral of a pointwise-median speed curve; lap 0 remains standing-start and excluded. Interactions remain unresolved.

### Controlled ablation status

| Treatment | Held-fixed identities | Driven time | Six-channel metrics | Status / limitations |
|---|---|---:|---|---|
| Controller legacy vs final model-aware | Identity unknown | not run | not run | Inconclusive: no identity-matched control pair. |
| Fixed vs optimized line | Identity unknown | not run | not run | Inconclusive: Task9 paired comparisons identity-inconclusive. |
| Battery initialisation response | Identity unknown | not run | not run | Current/state inputs unavailable; terminal voltage is not OCV. |
| Tire-force scale 0.95 / 1.05 | Identity unknown | not run | not run | No rebuilt/audited maps and replans; replay-only is robustness only. |
| Alignment ±1 bin and leave-one-lap-out | Physical lap times unchanged | not run | not run | Source geometry/provenance inconclusive; no best-shift correction. |

No causal effects are added or summed; interactions remain unresolved.

## Caveats

- **These are the values we did not measure** for this car: wheelbase and track width confirmation for this log, CG height and yaw/wheel inertia, tire force/slip and transient-tire parameters, battery capacity/resistance map, motor/drivetrain efficiency, brake and steering dynamics, air density, aero coefficients/balance, and brake bias. They are carried over/model calibrations, not identified from this stint.
- Raw-source provenance is verified by SHA-256 when `~/Desktop/0615CompEndurance.mat` is accessible; the generator hashes it when accessible but does not re-extract it because comparison uses the already-extracted cache. Cache SHA-256 is `{summary['measured']['cache_sha256']}` and extraction hash is `{summary['measured']['extraction_sha256']}`. If the raw source is unavailable on another machine, that is disclosed in metrics rather than blocking use of the cache.
- The saved record has SHA-256 `{sim['record_sha256']}` ({sim['record_bytes']} bytes). Its manifest identifies controller/track content but not a configuration-source hash; no claim is made that the current XML is embedded in that historical record.
- The record says `lap_completed=true`; the replay is strict-solver feasible, not an optimum. Its saved yaw-profile artifact records eight refinement rounds and resolution convergence; that 1 m planning-profile status does not certify a continuous mesh or this forward replay as optimal. Containment is sampled-centre contained, while footprint and between-step containment are not certified.
- Measurement position is dead-reckoned from velocity. Residual centreline offset, the battery initial-condition mismatch, and request-versus-delivered torque convention are separate from structural model error.
- The track XML metadata traces its reference geometry to the same MAT source (SHA-256 `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`); it is reconstructed from this log rather than an independent surveyed edge geometry. The 2.25 m assumed corridor is likewise not surveyed, so this is not independent geometry validation.

## Reproduce

```bash
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config \\
  python3 tools/run/run_with_deadline.py --seconds 300 -- \\
  python3 tools/reports/compare_yaw_mis_replay.py \\
  --record runs/yaw_lap_suite/mis-modelaware-matchedpreview-validated/run_record.npz \\
  --plan runs/yaw_lap_suite/mis-fixed-1m/plan.npz \\
  --measured-dir docs/validation/iem24-murphy \\
  --output-dir docs/validation/iem24-murphy-yaw-closure
```

Generated by `tools/reports/compare_yaw_mis_replay.py` following the [MIS 2024 report standard](../../standards/mis-2024-report-standard.md). The command consumes one saved record and cached measured telemetry; it does not rerun physics or OCP.
'''
 (out/'report.md').write_text(text,encoding='utf8')
def main():
 p=argparse.ArgumentParser();p.add_argument('--record',type=Path,required=True);p.add_argument('--measured-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--plan',type=Path,default=Path('runs/yaw_lap_suite/mis-fixed-1m/plan.npz'));p.add_argument('--torque-limit',type=float,default=LIMIT);p.add_argument('--track',type=Path,default=Path('models/tracks/mis_ev_endurance_2024_width.xml'));a=p.parse_args();out=a.output_dir;out.mkdir(parents=True,exist_ok=True)
 measured=load_table(a.measured_dir/'measured_stint.csv');ex=json.loads((a.measured_dir/'telemetry_extraction.json').read_text());track=load_track(ROOT/a.track);path=track.path_coordinates;L=path.lap_length;laps=flying_lap_indices(measured);off,fit0=fit_position_offset(measured,path);td,res=project_onto_track(measured,path,off);grid,folded=fold_laps(measured,laps,tuple(x[0] for x in CHANNELS),track_distance=td,lap_length=L,bins=BINS);med={k:np.median(v,0) for k,v in folded.items()};r,tu,d,reference_d,raw,accepted,filtered,sim,physical_length,reference_length,plan_physical,plan_reference=record_profile(a.record,a.plan,grid,L,torque_limit=a.torque_limit)
 lap=float(r.time[-1]-r.time[0]);# Lap duration is evidence from this record, not an admissibility constant.
 sim_cum=[]
 measured_mask=measured['lap_index']==laps[0]; measured_time=np.asarray(measured['time_s'][measured_mask],float)
 measured_physical=unwrap_track_distance(np.asarray(td[measured_mask],float),L,measured['lap_index'][measured_mask])
 try:
  measured_elapsed,measured_physical=prepare_full_lap(measured_time,measured_physical,L,start_time=float(ex['laps'][int(laps[0])]['start_s']),boundary_duration=float(ex['laps'][int(laps[0])]['lap_time_s']))
  measured_ref=map_lap_through_plan(measured_physical,plan_physical,plan_reference,L)
  sim_elapsed,sim_physical=prepare_full_lap(tu,d,physical_length,start_time=float(tu[0]))
  sim_ref=map_lap_through_plan(sim_physical,plan_physical,plan_reference,L)
  sector_rows=paired_sector_gap_table(sim_elapsed,sim_ref,measured_elapsed,measured_ref,L,sim_physical=sim_physical,meas_physical=measured_physical)
  sim_cum=cumulative_time_on_reference(sim_elapsed,sim_ref,sim_physical,grid,lap_length=L).tolist()
  sector_data={'status':'available','reference_lap_index':int(laps[0]),'rows':sector_rows,'largest_positive':sorted(sector_rows,key=lambda x:x['signed_gap_sim_minus_meas_s'],reverse=True)[:5],'largest_negative':sorted(sector_rows,key=lambda x:x['signed_gap_sim_minus_meas_s'])[:5],'measured_boundary_elapsed_s':float(measured_elapsed[-1]),'replay_boundary_elapsed_s':float(sim_elapsed[-1]),'mapping':'measured projected station unwrapped within lap; physical→reference map from saved plan'}
 except (ValueError,IndexError) as exc: sector_data={'status':'unavailable','reason':f'full-lap endpoint or saved-Jacobian mapping not established: {exc}'}
 trace_specs=[('requested_motor_torque_per_motor_nm',raw['motor_torque_request_nm'],'N·m',''),('delivered_rear_wheel_torque_nm',raw.get('delivered_wheel_torque_nm'),'N·m','wheel-drive-torque channel absent'),('electrical_bus_power_w',raw.get('bus_power_w'),'W','bus-power channel absent'),('electrical_power_limit_w',None,'W','no configured electrical-limit channel in saved record'),('pack_terminal_voltage_v',raw['pack_voltage_v'],'V',''),('battery_soc',raw['soc'],'1',''),('throttle_saturation',np.isclose(raw['throttle'],1.,atol=1e-6),'1',''),('brake_force_n',raw.get('brake_force_n'),'N','brake-force channel absent'),('brake_fraction',raw.get('brake_fraction'),'1','brake-fraction channel absent'),('brake_saturation',None,'1','actuator saturation state not recorded'),('steering_rate_rad_s',raw.get('steering_rate_rad_s'),'rad/s','steering-rate channel absent'),('steering_rate_saturation',None,'1','actuator saturation state not recorded')]
 operational={name:({'status':'available','unit':unit,'sample_count':int(np.asarray(vals).size),'values':np.asarray(vals).tolist()} if vals is not None else {'status':'unavailable','unit':unit,'reason':reason}) for name,vals,unit,reason in trace_specs}
 metrics={k:metric(sim[k],med[k]) for k,_,_ in CHANNELS}; rawmat=Path.home()/'Desktop/0615CompEndurance.mat';summary={'metrics':metrics,'ablations':[
   {'treatment':'Controller legacy vs final model-aware','status':'not run — no identity-matched legacy replay on identical saved fixed plan and initial plant state','held_fixed':'unknown / not identity-bound','driven_time_s':None,'six_channel_metrics':None,'limitations':'Historical provenance and paired comparisons are identity-inconclusive.'},
   {'treatment':'Fixed vs optimized line under final controller','status':'not run — no identity-matched controlled pair','held_fixed':'unknown / not identity-bound','driven_time_s':None,'six_channel_metrics':None,'limitations':'Task9 paired comparisons are identity-inconclusive.'},
   {'treatment':'Battery initialization response on one saved plan','status':'not run — current and state evidence needed for OCV/resistance inference is absent','held_fixed':'saved plan/plant identity unavailable for paired treatment','driven_time_s':None,'six_channel_metrics':None,'limitations':'Loaded terminal voltage is not OCV; causal battery attribution unresolved.'},
   {'treatment':'Grip scale 0.95 / 1.05 matched capability','status':'not run — no identity-bound maps/plans for perturbed tire model','held_fixed':'not established','driven_time_s':None,'six_channel_metrics':None,'limitations':'Requires rebuild, independent map audit, and replanning; replay-only would test robustness only.'},
   {'treatment':'Alignment ±1 bin and leave-one-lap-out translation','status':'not run — historical source provenance/geometry identity inconclusive','held_fixed':'physical lap times retained','driven_time_s':None,'six_channel_metrics':None,'limitations':'No best-shift correction applied; preserve original discrepancy.'}], 'measured':{'median_lap_time_s':float(np.median([x['lap_time_s'] for x in ex['laps'] if not x['standing_start']])), 'median_distance_m':float(np.median([x['distance_m'] for x in ex['laps'] if not x['standing_start']])), 'flying_laps_used':len(laps), 'extraction_sha256':sha(a.measured_dir/'telemetry_extraction.json'),'cache_sha256':sha(a.measured_dir/'measured_stint.csv'),'raw_sha256':sha(rawmat) if rawmat.is_file() else None,'raw_bytes':rawmat.stat().st_size if rawmat.is_file() else None},'simulation':{'lap_time_s':lap,'record_sha256':sha(a.record),'record_bytes':a.record.stat().st_size,'initial_soc':float(raw['soc'][0]),'initial_voltage_v':float(raw['pack_voltage_v'][0]),'final_soc':float(raw['soc'][-1]),'record_rows':r.rows,'physical_length_m':physical_length,'reference_length_m':reference_length,'plan_sha256':sha(a.plan),'lap_completed':r.manifest.segment.get('lap_completed'),'controller_identity':r.manifest.model.get('controller'),'yaw_profile_identity':r.manifest.model.get('yaw_profile',{}).get('array_digest'),'sector_time':sector_time_gaps(tu,reference_d,d,50.),'cumulative_time_reference_m':grid.tolist(),'cumulative_time_s':sim_cum,'initial_states':{name:float(r.state[0,i]) for i,name in enumerate(r.manifest.state_names)}},'sector_comparison':sector_data,'operational_traces':operational,'diagnostics':{'brake_onset_exit_speed':_not_available('measured cache contains no brake channel; no identity-matched brake onset reference'),'yaw_acceleration_residuals':_not_available('historical measurement/replay alignment and geometry provenance do not support residual attribution')},'position_fit':{'north':float(off[0]),'east':float(off[1]),'rmse':fit0,'median':float(np.median(res[np.isin(measured['lap_index'],laps)])),'p95':float(np.percentile(res[np.isin(measured['lap_index'],laps)],95))},'filter':{'simulation_resample_hz':100,'cutoff_hz':1,'order':1,'measured_double_filtered':False},'channels':{'simulation':{'speed':'speed','longitudinal_acceleration':'acceleration-longitudinal','lateral_acceleration':'acceleration-lateral','yaw_rate':'state:yaw_rate','motor_torque_request':f'throttle * {a.torque_limit} N*m reconstructed request; not wheel-drive-torque','pack_voltage':'bus-voltage'},'manifest_wheel_torque':'wheel-drive-torque = actual powertrain wheel torque, signed'}}
 (out/'metrics.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
 with (out/'operational_traces.csv').open('w',newline='\n') as f:
  w=csv.writer(f,lineterminator='\n'); w.writerow(['time_s',*operational]); [w.writerow([float(tu[i]),*[float(np.asarray(v['values'])[i]) if v['status']=='available' else '' for v in operational.values()]]) for i in range(len(tu))]

 with (out/'metrics.csv').open('w',newline='\n') as f:
  w=csv.writer(f,lineterminator='\n');w.writerow(['signal','rmse','mae','bias_sim_minus_meas','correlation','normalised_rmse']);[w.writerow([k,metrics[k]['rmse'],metrics[k]['mae'],metrics[k]['bias'],metrics[k]['correlation'],metrics[k]['normalised_rmse']]) for k,_,_ in CHANNELS]
 with (out/'ablation_status.json').open('w',encoding='utf8') as f: json.dump({'status':'inconclusive; no controlled causal comparisons run','gap_s':lap-summary['measured']['median_lap_time_s'],'gap_percent':100*(lap/summary['measured']['median_lap_time_s']-1),'treatments':summary['ablations'],'interactions':'unresolved'},f,indent=2,sort_keys=True);f.write('\\n')
 np.savetxt(out/'comparison.csv',np.column_stack([grid,*[z for k,_,_ in CHANNELS for z in (med[k],sim[k])]]),delimiter=',',header='track_distance_m,'+','.join(q for k,_,_ in CHANNELS for q in ('measured_'+k,'simulated_'+k)),comments='')
 fig(out,grid,L,folded,med,sim,tu,d,filtered,raw,accepted,r.time);report(out,summary,ex,L,torque_limit=a.torque_limit);print(json.dumps(metrics,indent=2))
if __name__=='__main__':main()
