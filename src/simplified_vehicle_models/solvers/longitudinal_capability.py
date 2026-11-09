"""Immutable offline sampled current-vehicle longitudinal capability.

Grid values are quasi-steady bounds using explicit path-aligned wheel-speed,
load-transfer, and point-mass friction-ellipse assumptions; not transient
vehicle/tire certification. Braking is reported as signed net progress accel.
"""
from __future__ import annotations
from dataclasses import dataclass, fields, is_dataclass
from hashlib import sha256
import json, math
from pathlib import Path
from typing import Protocol, runtime_checkable
import numpy as np

PEDAL_RESPONSE_MAP_VERSION = "signed-physical-pedal-response-v1"
# Performance-only cap: each source brake curve has at most seven active knots,
# so a union has at most 56 knots. Conservatively budget 20 KiB per complete
# Python-tuple cell pack (float objects, tuple slots/headers, and row containers);
# 1,024 packs therefore bound this optional cache-like static optimization to
# roughly 20 MiB. Larger custom grids simply use the exact legacy evaluator.
_BRAKE_CELL_INTERPOLANT_MAX_CELLS = 1024


def _canonical(value):
    """Complete deterministic structured payload (never repr-truncated)."""
    if value is None or isinstance(value,(str,bool,int)): return value
    if isinstance(value,(float,np.floating)):
        x=float(value)
        if not math.isfinite(x): return {"float":x.hex()}
        return x.hex()
    if isinstance(value,(np.integer,)): return int(value)
    if isinstance(value,np.ndarray): return {"ndarray":value.tolist(),"dtype":str(value.dtype),"shape":list(value.shape)}
    if isinstance(value,(tuple,list)): return [_canonical(x) for x in value]
    if isinstance(value,dict): return {str(k):_canonical(value[k]) for k in sorted(value,key=str)}
    if is_dataclass(value): return {"type":type(value).__qualname__,"fields":{f.name:_canonical(getattr(value,f.name)) for f in fields(value)}}
    if hasattr(value,"__dict__"): return {"type":type(value).__qualname__,"attrs":_canonical(vars(value))}
    slots=getattr(type(value),"__slots__",())
    if slots: return {"type":type(value).__qualname__,"slots":{k:_canonical(getattr(value,k)) for k in slots if isinstance(k,str) and hasattr(value,k)}}
    return {"type":type(value).__qualname__}

def _digest(obj): return sha256(json.dumps(_canonical(obj),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()

def _source_closure_hash(model_source_hashes=None):
    root=Path(__file__).resolve().parents[3]
    package=root/"src/simplified_vehicle_models"
    entries=[]
    for p in sorted(package.rglob("*.py")):
        name=p.relative_to(package).as_posix()
        entries.append((name,sha256(p.read_bytes()).hexdigest()))
    if not entries: raise ValueError("source closure unavailable")
    if model_source_hashes is not None:
        if not isinstance(model_source_hashes,dict) or not model_source_hashes: raise ValueError("model_source_hashes must be a nonempty path-to-SHA256 mapping")
        artifact=[]
        for rel,digest in sorted(model_source_hashes.items()):
            if not isinstance(rel,str) or not rel or "\\" in rel:
                raise ValueError("model source path must be a canonical repository-relative POSIX path")
            key=Path(rel)
            if key.is_absolute() or key.as_posix()!=rel or any(part in ("", ".", "..") for part in rel.split("/")):
                raise ValueError("model source path must be a canonical repository-relative POSIX path")
            lexical=root/key
            # lstat every existing component so both a symlink file and a
            # symlinked ancestor are rejected, even if their target stays in root.
            cursor=root
            linked=False
            for part in key.parts:
                cursor=cursor/part
                try:
                    if cursor.is_symlink(): linked=True; break
                except OSError:
                    linked=True; break
            path=lexical.resolve()
            if linked or root not in path.parents or not path.is_file():
                raise ValueError("model source path must be a repository-rooted regular file without symlink aliases")
            actual=sha256(path.read_bytes()).hexdigest()
            if not isinstance(digest,str) or digest!=actual: raise ValueError(f"model source hash mismatch: {rel}")
            artifact.append((rel,actual))
        return _digest({"code":entries,"model_sources":artifact})
    return _digest({"code":entries,"model_sources":None})

@dataclass(frozen=True, slots=True)
class CapabilityQuery:
    speed_mps: float
    curvature_inv_m: float
    soc: float
    yaw_moment_request_nm: float=0.
    def __post_init__(self):
        if not all(math.isfinite(float(x)) for x in (self.speed_mps,self.curvature_inv_m,self.soc,self.yaw_moment_request_nm)): raise ValueError("capability query values must be finite")
        if self.speed_mps<0 or not 0<=self.soc<=1: raise ValueError("speed must be nonnegative and SOC in [0, 1]")

@dataclass(frozen=True, slots=True)
class LongitudinalCapability:
    drive_net_progress_acceleration_mps2: float
    coast_net_progress_acceleration_mps2: float
    brake_net_progress_acceleration_mps2: float
    in_domain: bool
    limiting_constraints: tuple[str,...]
    allocator_identity: str
    drive_pedal_grid: tuple[float,...] | None = None
    drive_acceleration_grid: tuple[float,...] | None = None
    brake_pedal_grid: tuple[float,...] | None = None
    brake_acceleration_grid: tuple[float,...] | None = None
    brake_pedal_threshold: float | None = None

@runtime_checkable
class CapabilityProvider(Protocol):
    identity: str
    def evaluate(self,query:CapabilityQuery)->LongitudinalCapability: ...

def validate_execution_allocator(capability_identity,execution_identity,yaw_moment_request_nm=0.,allocator_provider=None):
    if not math.isfinite(float(yaw_moment_request_nm)): raise ValueError("yaw request must be finite")
    provider_identity=getattr(allocator_provider,"identity",None) if allocator_provider is not None else capability_identity
    if not (capability_identity==execution_identity==provider_identity): raise ValueError("torque allocation provider has no matching execution path")
    if yaw_moment_request_nm!=0 and allocator_provider is None: raise ValueError("torque allocation provider has no matching execution path")

@dataclass(frozen=True,slots=True)
class PathKinematicContext:
    vx_mps:float; vy_mps:float; heading_error_rad:float; lateral_error_m:float
    yaw_rate_rps:float; curvature_inv_m:float; curvature_gradient_inv_m2:float
    vy_dot_estimate_mps2:float; vy_dot_valid:bool
    def __post_init__(self):
        vals=(self.vx_mps,self.vy_mps,self.heading_error_rad,self.lateral_error_m,self.yaw_rate_rps,self.curvature_inv_m,self.curvature_gradient_inv_m2,self.vy_dot_estimate_mps2)
        if not all(math.isfinite(float(x)) for x in vals): raise ValueError("kinematic context must be finite")
        if not isinstance(self.vy_dot_valid,bool): raise TypeError("vy_dot_valid must be bool")

def _frenet(context):
    e=context.heading_error_rad;c=math.cos(e);s=math.sin(e);k=context.curvature_inv_m
    B=1-k*context.lateral_error_m
    if not context.vy_dot_valid: raise ValueError("vy_dot estimate is not valid")
    if B<.5 or abs(c)<1e-6: raise ValueError("kinematic context outside Frenet conversion domain")
    U=context.vx_mps*c-context.vy_mps*s;p=U/B
    nd=context.vx_mps*s+context.vy_mps*c;ed=context.yaw_rate_rps-k*p
    Bd=-context.curvature_gradient_inv_m2*p*context.lateral_error_m-k*nd
    return c,s,B,p,nd,ed,Bd

def path_progress_to_body_vx_dot(a,context):
    a=float(a)
    if not math.isfinite(a): raise ValueError("target acceleration must be finite")
    c,s,B,p,nd,ed,Bd=_frenet(context)
    return (B*a+context.vy_dot_estimate_mps2*s+nd*ed+p*Bd)/c

def body_vx_dot_to_path_progress(x,context):
    x=float(x)
    if not math.isfinite(x): raise ValueError("vx_dot must be finite")
    c,s,B,p,nd,ed,Bd=_frenet(context)
    return (x*c-context.vy_dot_estimate_mps2*s-nd*ed-p*Bd)/B

def _validated_physical_curve(grid,values,*,braking=False,capability=None):
    if grid is None or values is None: raise ValueError("unsupported_missing_physical_pedal_map")
    if len(grid)<2 or len(grid)!=len(values): raise ValueError("invalid_active_pedal_map")
    g=tuple(float(x) for x in grid); y=tuple(float(x) for x in values)
    if not all(math.isfinite(x) for x in (*g,*y)) or any(b<=a for a,b in zip(g,g[1:])): raise ValueError("invalid_active_pedal_map")
    if g[0]!=0. or g[-1]!=1.: raise ValueError("invalid_active_pedal_map")
    if braking and any(b>a+1e-10 for a,b in zip(y,y[1:])): raise ValueError("invalid_active_pedal_map: brake response is nonmonotone")
    if not braking and any(b<a-1e-10 for a,b in zip(y,y[1:])): raise ValueError("invalid_active_pedal_map: drive response is nonmonotone")
    if capability is not None:
        if not math.isclose(y[0],capability.coast_net_progress_acceleration_mps2,rel_tol=0.,abs_tol=1e-8): raise ValueError("invalid_active_pedal_map: coast endpoint mismatch")
        endpoint=capability.brake_net_progress_acceleration_mps2 if braking else capability.drive_net_progress_acceleration_mps2
        if not math.isclose(y[-1],endpoint,rel_tol=0.,abs_tol=1e-8): raise ValueError("invalid_active_pedal_map: full-pedal endpoint mismatch")
    return g,y

def _curve_value(grid,values,p):
    return float(np.interp(p,grid,values))

def evaluate_pedal_response(capability,pedal_position):
    """Interpolate the immutable stored physical normalized-pedal curve."""
    p=float(pedal_position)
    if not math.isfinite(p) or not -1.<=p<=1.: raise ValueError("physical pedal position must be finite in [-1,1]")
    if not capability.in_domain: raise ValueError("pedal response outside capability domain")
    if p>=0.: grid,values=capability.drive_pedal_grid,capability.drive_acceleration_grid;p=abs(p)
    else: grid,values=capability.brake_pedal_grid,capability.brake_acceleration_grid;p=abs(p)
    grid,values=_validated_physical_curve(grid,values,braking=pedal_position<0.,capability=capability)
    return _curve_value(grid,values,p)

def pedal_for_acceleration(capability,target_acceleration):
    """Invert the same validated monotone physical brake map."""
    target=float(target_acceleration)
    if not math.isfinite(target): raise ValueError("target acceleration must be finite")
    if not capability.in_domain: raise ValueError("pedal response outside capability domain")
    grid,values=_validated_physical_curve(capability.brake_pedal_grid,capability.brake_acceleration_grid,braking=True,capability=capability)
    if max(values)-min(values)<=1.e-12: raise ValueError("zero_brake_authority")
    if not min(values)<=target<=max(values): raise ValueError("brake target outside physical pedal-map authority")
    # np.interp expects increasing x; reversed response is nondecreasing. Collapse
    # legitimate saturated duplicates while rejecting all malformed active nodes.
    xx=[]; yy=[]
    for value,pedal in zip(values[::-1],grid[::-1]):
        if xx and value==xx[-1]: yy[-1]=pedal
        else: xx.append(value); yy.append(pedal)
    return -float(np.interp(target,xx,yy))

def _stored_brake_grid(grid,values):
    """Strip only equal-pair padding; duplicate/descending active nodes remain invalid."""
    g=tuple(float(x) for x in grid);v=tuple(float(x) for x in values)
    while len(g)>2 and g[-1]==g[-2] and v[-1]==v[-2]: g=g[:-1];v=v[:-1]
    return g,v

def _interp_stored_curve(grid,values,p):
    g,v=tuple(grid),tuple(values)
    if len(g)!=len(v) or len(g)<2: raise ValueError("invalid_active_pedal_map")
    return float(np.interp(p,g,v))

def _readonly_bytes_array(array):
    owned=np.asarray(array)
    return np.frombuffer(owned.tobytes(order="C"),dtype=owned.dtype).reshape(owned.shape)

def _readonly_reason_tuples(array):
    return tuple(tuple(tuple(row) for row in plane) for plane in array.tolist())

class CurrentVehicleCapability:
    """Immutable sampled provider; optional caller capacities only derate model estimates."""
    def __setattr__(self,n,v):
        if getattr(self,"_frozen",False): raise AttributeError("CurrentVehicleCapability is immutable")
        object.__setattr__(self,n,v)
    def __init__(self,vehicle,*,wheel_radii_m,speed_grid_mps,curvature_grid_inv_m,soc_grid,source_label,calibration_label,normal_loads_n=None,longitudinal_capacity_n=None,expected_source_hash=None,model_source_hashes=None):
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext,BrakeAuthorityPredictor
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        from simplified_vehicle_models.solvers.vertical_forward import _drivetrain_context
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar,normal_loads,tire_force_capacities,_allocate_axle_force
        self._radii=tuple(float(x) for x in wheel_radii_m)
        if len(self._radii)!=4 or not all(math.isfinite(x) and x>0 for x in self._radii): raise ValueError("wheel radii must be finite positive FL/FR/RL/RR")
        self.speed_grid=tuple(float(x) for x in speed_grid_mps);self.curvature_grid=tuple(float(x) for x in curvature_grid_inv_m);self.soc_grid=tuple(float(x) for x in soc_grid)
        for nm,g in (("speed",self.speed_grid),("curvature",self.curvature_grid),("SOC",self.soc_grid)):
            if len(g)<2 or not all(math.isfinite(x) for x in g) or any(y<=x for x,y in zip(g,g[1:])): raise ValueError(f"{nm} grid must be finite, strictly increasing, length >=2")
        if self.speed_grid[0]<0 or self.soc_grid[0]<0 or self.soc_grid[-1]>1: raise ValueError("invalid speed or SOC grid")
        if normal_loads_n is not None and (len(normal_loads_n)!=4 or not all(math.isfinite(float(x)) and float(x)>0 for x in normal_loads_n)): raise ValueError("normal load estimates must be finite positive four-tuples")
        if longitudinal_capacity_n is not None and (len(longitudinal_capacity_n)!=4 or not all(math.isfinite(float(x)) and float(x)>=0 for x in longitudinal_capacity_n)): raise ValueError("optional capacity derates must be finite nonnegative four-tuples")
        if vehicle.controller.regen_enabled: raise ValueError("regenerative braking is unsupported")
        if not source_label or not calibration_label: raise ValueError("source and calibration descriptive labels required")
        self.source_label=str(source_label);self.calibration_label=str(calibration_label)
        self.source_closure_hash=_source_closure_hash(model_source_hashes)
        if expected_source_hash is not None and expected_source_hash!=self.source_closure_hash: raise ValueError("source closure hash mismatch")
        # Capture complete canonical payload and discard every reference to model.
        payload=_canonical(vehicle); self.model_hash=_digest(payload)
        self.calibration_hash=_digest({"controller":_canonical(vehicle.controller),"hydraulics":{k:_canonical(getattr(vehicle,k,None)) for k in ("maximum_pedal_force","pedal_curve","hydraulic_ratio_front","hydraulic_ratio_rear","maximum_brake_pressure","caliper_area","brake_bias","pad_mu_front","pad_mu_rear","brake_effective_radius_front","brake_effective_radius_rear")}})
        self.source_identity_hash=self.source_closure_hash
        self.provenance_payload=json.dumps(payload,sort_keys=True,separators=(",",":"),allow_nan=False)
        self.identity=_digest({"model":self.model_hash,"source":self.source_identity_hash,"calibration":self.calibration_hash,"grids":[self.speed_grid,self.curvature_grid,self.soc_grid],"radii":self._radii,"normal_estimate":normal_loads_n,"capacity_derate":longitudinal_capacity_n,"assumptions":"pointmass_load_transfer_3fixed_point_iterations;path_aligned_v_over_r;ellipse;common_hydraulic_prefix;net_signed_acceleration;"+PEDAL_RESPONSE_MAP_VERSION+";drive_grid_0_.125_.25_.5_.75_1"})
        view=point_mass_view_of_planar(vehicle)
        driven,gearing,eff=_drivetrain_context(vehicle)
        bp=BrakeAuthorityPredictor(vehicle,maximum_abs_curvature_inv_m=max(abs(self.curvature_grid[0]),abs(self.curvature_grid[-1])),minimum_speed_mps=max(.01,self.speed_grid[0]),maximum_speed_mps=max(self.speed_grid[-1],self.speed_grid[0]+.01))
        if not bp._linear_fast_path:
            raise ValueError("unsupported shaped hydraulic contact-prefix capability map")
        shape=(len(self.speed_grid),len(self.curvature_grid),len(self.soc_grid));table=np.zeros(shape+(3,));valid=np.ones(shape,dtype=bool);reasons=np.empty(shape,dtype=object);drive_maps=np.zeros(shape+(6,));brake_maps=np.zeros(shape+(7,));brake_grids=np.zeros(shape+(7,));brake_tau=np.ones(shape);drive_grid=(0.,.125,.25,.5,.75,1.)
        mass=float(vehicle.total_mass);J=float(vehicle.wheel_inertia);r=np.asarray(self._radii)
        for i,v in enumerate(self.speed_grid):
          for j,k in enumerate(self.curvature_grid):
            lateral=v*v*k; loads_hint=np.asarray(normal_loads_n,dtype=float) if normal_loads_n is not None else None
            for m,soc in enumerate(self.soc_grid):
              why=[]; low=max(v,.01); speeds=low/r
              aero_drag=.5*vehicle.aero.air_density*vehicle.aero.drag_area*v*v
              effmass=mass+J*float(np.sum(1/r**2))
              # At most three fixed-point updates of longitudinal load transfer.
              estimates=[]
              for accel in (0.,): estimates.append(accel)
              drive=0.;brake_acc=0.;node_ok=True;drive_values=();brake_values=();brake_grid=()
              for mode in ("drive","brake","coast"):
                acc=0.
                for _ in range(3):
                  loads=normal_loads(view,v,acc,lateral)
                  if loads_hint is not None: loads=np.minimum(loads,loads_hint)
                  if not np.all(np.isfinite(loads)) or np.any(loads<=0): node_ok=False;why.append("nonpositive_contact_load");break
                  lc,latc=tire_force_capacities(view,loads)
                  lf=np.zeros(4)
                  front_ok=_allocate_axle_force(lf,latc,slice(0,2),mass*abs(lateral)*view.front_mass_fraction)
                  rear_ok=_allocate_axle_force(lf,latc,slice(2,4),mass*abs(lateral)*(1-view.front_mass_fraction))
                  if not front_ok or not rear_ok:
                    node_ok=False;why.append("lateral_demand_exceeds_axle_capacity")
                  if not getattr(view.tire,"longitudinal_force_scale",1.) or not getattr(view.tire,"lateral_force_scale",1.):
                    node_ok=False;why.append("unsupported_zero_tire_capacity")
                  ratio=np.divide(lf,latc,out=np.zeros(4),where=latc>0)
                  rem=lc*np.sqrt(np.maximum(0.,1-ratio*ratio))
                  if longitudinal_capacity_n is not None: rem=np.minimum(rem,np.asarray(longitudinal_capacity_n,dtype=float))
                  if not np.all(np.isfinite(rem)): node_ok=False;why.append("invalid_tire_capacity");break
                  if np.any(rem<=0) and mode!="coast": why.append("zero_remaining_tire_capacity")
                  if mode=="drive":
                    # Physical throttle samples are run through the real scalar powertrain closure;
                    # apply one common tire prefix after each wheel's force has been computed.
                    curve=[]
                    for pedal in drive_grid:
                      pt=evaluate_forward_powertrain(vehicle,driven,gearing,eff,speeds,soc,DriverCommand(pedal,0.,0.));force=np.asarray(pt.motor_wheel_torques)/r
                      prefix=min(1.,float(np.min(np.divide(rem,force,out=np.full(4,np.inf),where=force>0))))
                      coast_ctx=BrakeAuthorityContext(low,k,self._radii,tuple(float(x) for x in loads),tuple(float(x) for x in rem),soc=soc)
                      resistance=bp.predict(coast_ctx,0.).deceleration_mps2*effmass
                      curve.append((float(np.sum(force))*prefix-resistance)/effmass)
                    drive_values=tuple(curve);acc=curve[-1]
                  elif mode=="coast":
                    # zero-throttle motor torque plus aero and exact tire rolling estimate
                    from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
                    ctx=BrakeAuthorityContext(low,k,self._radii,tuple(float(x) for x in loads),tuple(float(x) for x in rem),soc=soc)
                    coast_pred=bp.predict(ctx,0.)
                    acc=-coast_pred.deceleration_mps2
                  else:
                    from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
                    ctx=BrakeAuthorityContext(low,k,self._radii,tuple(float(x) for x in loads),tuple(float(x) for x in rem),soc=soc)
                    full=bp.predict(ctx,vehicle.maximum_pedal_force)
                    if not full.in_domain: node_ok=False;why.append("brake_authority_outside_domain")
                    # The predictor's returned pedal_force_n is the exact physical
                    # force corresponding to the tire/contact-capped endpoint.
                    tau=float(full.pedal_force_n)/float(vehicle.maximum_pedal_force)
                    if tau <= 1.e-12:
                      why.append("zero_brake_authority"); brake_grid=(0.,1.); brake_values=(-bp.predict(ctx,0.).deceleration_mps2,)*2
                    else:
                      brake_grid=tuple(sorted(set((0.,.125,.25,.5,.75,1.,tau))))
                      brake_values=tuple(-bp.predict(ctx,p*vehicle.maximum_pedal_force).deceleration_mps2 for p in brake_grid)
                    brake_tau[i,j,m]=tau;acc=brake_values[-1]
                  if longitudinal_capacity_n is not None: why.append("caller_capacity_derate_applied")
                if mode=="drive": drive=acc
                elif mode=="coast": coast=acc
                else: brake_acc=acc
              table[i,j,m,:]=(drive,coast,brake_acc);valid[i,j,m]=node_ok;reasons[i,j,m]=tuple(dict.fromkeys(why)) or ("pointmass_load_transfer_and_tire_ellipse",)
              drive_maps[i,j,m,:]=drive_values or (coast,)*6
              bg=brake_grid or (0.,1.);bv=brake_values or (coast,brake_acc)
              # Fixed-size storage is padded only after the actual last node; its
              # per-node grid is separately interpolated and identity-bound.
              bg=tuple(bg)+(bg[-1],)*(7-len(bg));bv=tuple(bv)+(bv[-1],)*(7-len(bv))
              brake_grids[i,j,m,:]=bg;brake_maps[i,j,m,:]=bv
        table=_readonly_bytes_array(table);valid=_readonly_bytes_array(valid);drive_maps=_readonly_bytes_array(drive_maps);brake_maps=_readonly_bytes_array(brake_maps);brake_grids=_readonly_bytes_array(brake_grids);brake_tau=_readonly_bytes_array(brake_tau)
        reasons=_readonly_reason_tuples(reasons)
        # Immutable per-node brake curves avoid parsing padded storage on every
        # query. Tuple-backed metadata preserves the provider's frozen contract.
        brake_curves=[]
        for index in np.ndindex(shape):
            g,v=_stored_brake_grid(brake_grids[index],brake_maps[index])
            brake_curves.append((tuple(g),tuple(v)))
        self._brake_curves=tuple(brake_curves)
        # Full-cell packs are a performance-only optional precompute. Bound their
        # worst-case allocation before building any pack; above the cap, evaluate
        # uses the exact original union/interpolation algorithm without physical
        # validation/domain changes.
        cell_count=(shape[0]-1)*(shape[1]-1)*(shape[2]-1)
        if cell_count<=_BRAKE_CELL_INTERPOLANT_MAX_CELLS:
          cell_packs=[]
          for i in range(shape[0]-1):
            for j in range(shape[1]-1):
              for m in range(shape[2]-1):
                indices=tuple((i+((mask>>0)&1),j+((mask>>1)&1),m+((mask>>2)&1)) for mask in range(8))
                curves=tuple(self._brake_curves[(ix[0]*shape[1]+ix[1])*shape[2]+ix[2]] for ix in indices)
                grid=tuple(sorted(set(knot for curve,_ in curves for knot in curve)))
                rows=tuple(tuple(float(x) for x in np.interp(grid,g,v)) for g,v in curves)
                cell_packs.append((grid,rows))
          self._brake_cell_interpolants=tuple(cell_packs)
        else:
          self._brake_cell_interpolants=None
        self._brake_curve_shape=shape
        self._table=table;self._valid=valid;self._reasons=reasons;self._drive_maps=drive_maps;self._brake_maps=brake_maps;self._brake_grids=brake_grids;self._brake_tau=brake_tau;self._drive_pedal_grid=drive_grid
        self._frozen=True
    def evaluate(self,query):
        if not isinstance(query,CapabilityQuery): raise TypeError("query must be CapabilityQuery")
        if query.yaw_moment_request_nm!=0: raise ValueError("torque allocation provider has no matching execution path")
        axes=(self.speed_grid,self.curvature_grid,self.soc_grid);vals=(query.speed_mps,query.curvature_inv_m,query.soc)
        if not all(a[0]<=x<=a[-1] for a,x in zip(axes,vals)): return LongitudinalCapability(0.,0.,0.,False,("outside_grid_domain",),self.identity)
        lo=[];f=[]
        for a,x in zip(axes,vals):
            h=min(max(int(np.searchsorted(a,x,side="right")-1),0),len(a)-2);lo.append(h);f.append((x-a[h])/(a[h+1]-a[h]))
        out=np.zeros(3);tau=0.;supports=[]
        for mask in range(8):
            ix=tuple(lo[d]+((mask>>d)&1) for d in range(3));w=math.prod(f[d] if (mask>>d)&1 else 1-f[d] for d in range(3))
            if w>1e-15: supports.append((ix,w));out+=w*self._table[ix];tau+=w*self._brake_tau[ix]
        ok=all(bool(self._valid[ix]) for ix,w in supports);reasons=tuple(dict.fromkeys(reason for ix,w in supports for reason in self._reasons[ix[0]][ix[1]][ix[2]]))
        # Common physical-pedal union is exact for weighted piecewise-linear
        # maps. Vector interpolation retains support iteration and sum order.
        drive_grid=self._drive_pedal_grid
        drive_values_array=np.zeros(len(drive_grid))
        for ix,w in supports:
            drive_values_array+=w*self._drive_maps[ix[0],ix[1],ix[2],:]
        drive_values=tuple(float(x) for x in drive_values_array)
        flat=lambda ix:(ix[0]*self._brake_curve_shape[1]+ix[1])*self._brake_curve_shape[2]+ix[2]
        if len(supports)==8 and self._brake_cell_interpolants is not None:
            cell=(lo[0]*(self._brake_curve_shape[1]-1)+lo[1])*(self._brake_curve_shape[2]-1)+lo[2]
            brake_grid,rows=self._brake_cell_interpolants[cell]
            brake_values_array=np.zeros(len(brake_grid))
            for row,(ix,w) in zip(rows,supports):
                brake_values_array+=w*np.asarray(row)
        else:
            brake_curves=[self._brake_curves[flat(ix)] for ix,w in supports]
            brake_grid=tuple(sorted(set(knot for grid,values in brake_curves for knot in grid)))
            brake_values_array=np.zeros(len(brake_grid))
            for (grid,values),(ix,w) in zip(brake_curves,supports):
                brake_values_array+=w*np.interp(brake_grid,grid,values)
        brake_values=tuple(float(x) for x in brake_values_array)
        if not ok: reasons=tuple(dict.fromkeys(reasons+("unsupported_interpolation_node",)))
        return LongitudinalCapability(*map(float,out),ok,reasons,self.identity,drive_grid,drive_values,brake_grid,brake_values,float(tau))
