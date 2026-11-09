"""Shared bounded setup for the opt-in hybrid drive/brake policy.

This module intentionally contains setup orchestration only. Capability, corner
limits, immutable planning, and policy rollout remain implemented by their
existing shared solver APIs.
"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
import math

import numpy as np

HYBRID_SETUP_VERSION = "hybrid-shared-setup-v4-verified-continuation-relay"
HYBRID_POLICY_MAX_EXTRA_FORECASTS = 8
HYBRID_POLICY_BRACKET_RESOLUTION = 1e-8
HYBRID_POLICY_MAX_TOTAL_FORECASTS = 11
HYBRID_MODE = "hybrid-drive-brake-v1"
HYBRID_EXECUTION_ALLOCATOR_ID = "independent-wheel-torque-v1"
HYBRID_HORIZON_S = 3.0
HYBRID_SLABS = 30
HYBRID_DOMAIN_CAP_RESERVE_SCHEMA = "first-invalid-cell-minus-1e-9-relative-v1"


def _immutable_float_array(values):
    array = np.asarray(values, dtype=np.float64)
    return np.frombuffer(array.tobytes(order="C"), dtype=np.float64).reshape(array.shape)


def _digest_payload(payload):
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return sha256(encoded).hexdigest(), encoded.decode("utf-8")


@dataclass(frozen=True, slots=True)
class HybridSetup:
    provider: object
    plan: object
    policy: object
    brake_predictor: object
    pedal_context: object
    corner_speed_caps_mps: np.ndarray
    physical_corner_speed_caps_mps: np.ndarray
    capability_domain_speed_caps_mps: np.ndarray
    setup_identity: str
    setup_identity_payload_json: str
    builder_query_log: tuple


def build_hybrid_setup(vehicle, *, distance_m, curvature_inv_m, maximum_speed_mps,
                       initial_speed_mps, storage_soc, wheel_radii_m, model_source_hashes,
                       maximum_pedal_force_n, pedal_context, throttle_acceleration_scale=2.5,
                       lateral_margin=.9, brake_margin=.60, is_closed=False, yaw_moment_request_nm=0.,
                       execution_allocator_identity=None):
    """Build one coherent 30-slab provider/plan/policy from actual model input.

    Invalid capability cells are never repaired or clamped. The grid is bounded
    to sampled trajectory curvatures plus the zero-curvature anchor, and speed
    samples are restricted per curvature to independently computed pointwise
    physical corner caps. Hybrid planning performs the only reachability pass. This is
    operational sampled capability, not certification.
    """
    from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
    from simplified_vehicle_models.formulations.point_mass import (
        maximum_corner_speed, point_mass_view_of_planar,
    )
    from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
    from simplified_vehicle_models.powertrain.energy_storage import StorageState
    from simplified_vehicle_models.solvers.hybrid_driver import (
        HYBRID_POLICY_VERSION, HybridLongitudinalPolicy, HybridPolicyConfig,
    )
    from simplified_vehicle_models.solvers.hybrid_speed_plan import HYBRID_PLAN_VERSION, build_hybrid_speed_plan
    from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability, _source_closure_hash

    # Source closure is computed by the shared capability module; this setup
    # orchestration module is deliberately not special-cased or omitted.

    if yaw_moment_request_nm != 0.:
        raise ValueError("nonzero yaw request unsupported: no matching real execution allocator")
    if execution_allocator_identity not in (None, HYBRID_EXECUTION_ALLOCATOR_ID):
        raise ValueError("hybrid setup requires the actual non-TV execution allocator identity")
    actual_allocator=execution_allocator_identity or HYBRID_EXECUTION_ALLOCATOR_ID
    if getattr(getattr(vehicle, "controller", None), "regen_enabled", True):
        raise ValueError("hybrid setup requires regenerative braking disabled")
    if getattr(getattr(vehicle, "controller", None), "actuator_calibration", None) is not None:
        raise ValueError("hybrid setup does not support nonlinear actuator calibration")
    if maximum_pedal_force_n <= 0 or not math.isfinite(float(maximum_pedal_force_n)):
        raise ValueError("compiled model must provide positive maximum pedal force")
    if not math.isfinite(float(initial_speed_mps)) or float(initial_speed_mps) < 0.:
        raise ValueError("initial speed must be finite and nonnegative")
    if not math.isfinite(float(throttle_acceleration_scale)) or float(throttle_acceleration_scale) <= 0.:
        raise ValueError("throttle acceleration scale must be finite and positive")
    if not math.isfinite(float(brake_margin)) or not 0. < float(brake_margin) <= 1.:
        raise ValueError("brake margin must be finite and in (0, 1]")
    x=np.asarray(distance_m,dtype=float); k=np.asarray(curvature_inv_m,dtype=float)
    if x.ndim!=1 or k.ndim!=1 or x.size<3 or x.size!=k.size or not np.all(np.isfinite(x)) or not np.all(np.isfinite(k)) or np.any(np.diff(x)<=0):
        raise ValueError("hybrid path grid must be finite, signed, and strictly increasing")
    if x[0] != 0. or not np.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0:
        raise ValueError("hybrid setup needs a zero-origin path and positive finite speed cap")
    if not 0. <= storage_soc <= 1. or not 0. < lateral_margin <= 1.:
        raise ValueError("invalid storage SOC or lateral margin")
    # Fixed bounded signed-curvature axis: preserve exact extrema and zero,
    # distribute remaining knots uniformly over the signed range. The capability
    # grid is a sampling model, not a claim to exact values at every track node.
    kmin=min(float(np.min(k)),0.0); kmax=max(float(np.max(k)),0.0)
    if math.isclose(kmin,kmax,rel_tol=0.,abs_tol=1e-12):
        eps=max(1e-9,abs(kmin)*1e-6); curvature_grid=(kmin-eps,kmax+eps)
    else:
        curvature_grid=tuple(float(v) for v in np.linspace(kmin,kmax,17))
    # The planner conserves SOC exactly. Make the supplied SOC an interior
    # grid knot so its query has a single nonzero support; neighboring rows are
    # still real samples, while no endpoint SOC is fabricated or required.
    soc=float(storage_soc)
    soc_eps=min(.01,soc,1.-soc)
    if soc_eps<=0.:
        raise ValueError("hybrid capability requires interior storage SOC")
    soc_grid=(soc-soc_eps,soc,soc+soc_eps)
    radii=tuple(float(v) for v in wheel_radii_m)
    if len(radii) != 4 or not all(math.isfinite(v) and v > 0. for v in radii):
        raise ValueError("wheel radii must be four finite positive values")
    # Compute strictly pointwise physical corner caps. The speed planner's
    # profile also includes forward/backward reachability and must not narrow
    # the sampled capability domain. Apply the requested lateral margin exactly
    # once here; hybrid reachability is performed only by its builder below.
    point_mass = vehicle if isinstance(vehicle, PointMassVehicleParameters) else point_mass_view_of_planar(vehicle)
    state = StorageState(float(storage_soc))
    caps = np.asarray([
        min(float(maximum_speed_mps), maximum_corner_speed(
            point_mass,
            float(lateral_margin) / max(float(abs(curvature)), 1. / 1.0e4),
            maximum_speed=float(maximum_speed_mps), storage_state=state,
        ))
        for curvature in k
    ], dtype=float)
    physical_caps=caps.copy()
    if caps.size != x.size:
        raise ValueError("corner builder path grid mismatch")
    # Use at most 17 speed samples. Invalid cells remain untouched in provider.
    global_hi=max(float(maximum_speed_mps),float(initial_speed_mps),.005)
    speed_grid=tuple(float(v) for v in np.linspace(0.,global_hi,17))
    if not isinstance(model_source_hashes,dict) or not model_source_hashes:
        raise ValueError("hybrid setup requires nonempty repository-rooted source hashes")
    source_hashes=tuple(sorted((str(path),str(digest)) for path,digest in model_source_hashes.items()))
    provider=CurrentVehicleCapability(vehicle,wheel_radii_m=radii,speed_grid_mps=speed_grid,
        curvature_grid_inv_m=curvature_grid,soc_grid=soc_grid,source_label="compiled-model-source-closure",
        calibration_label="compiled-physical-pedal-calibration",model_source_hashes=model_source_hashes)
    # A query is valid only when every nonzero interpolation support node is
    # valid. For each signed path curvature, find the largest contiguous valid
    # speed prefix with complete SOC/curvature interpolation support, then reserve
    # one speed-grid cell below its upper edge. This limits domain only; it does
    # not alter provider cells or query coordinates.
    reserve=max(1e-9,1e-9*float(speed_grid[-1]))
    soc_index=1
    def complete_valid_speed_ceiling(curvature):
        ci=min(max(int(np.searchsorted(curvature_grid,curvature,side="right")-1),0),len(curvature_grid)-2)
        curvature_support=(ci,) if math.isclose(curvature,float(curvature_grid[ci]),rel_tol=0.,abs_tol=1e-14) else (ci,ci+1)
        ceiling=float(speed_grid[0])
        for si in range(len(speed_grid)-1):
            if np.all(provider._valid[si:si+2,curvature_support,soc_index]):
                ceiling=float(speed_grid[si+1])
            else:
                break
        return ceiling
    # Require complete valid cells at path endpoints and each edge midpoint.
    path_curvature=list(map(float,k))
    if is_closed:
        path_curvature=path_curvature[:-1]
    edge_limits=[]
    for i in range(len(path_curvature) if is_closed else len(path_curvature)-1):
        j=(i+1)%len(path_curvature)
        edge_limits.append((i,j,(path_curvature[i]+path_curvature[j])/2.))
    domain_caps=np.empty_like(caps)
    for i,curvature in enumerate(path_curvature):
        incident=[]
        for a,b,mid in edge_limits:
            if i in (a,b):
                incident.append(min(complete_valid_speed_ceiling(path_curvature[a]),complete_valid_speed_ceiling(mid),complete_valid_speed_ceiling(path_curvature[b])))
        bound=min(incident) if incident else complete_valid_speed_ceiling(curvature)
        domain_caps[i]=min(caps[i],max(0.,bound-reserve))
    if is_closed:
        domain_caps[-1]=domain_caps[0]
        caps[-1]=caps[0]
    caps=np.minimum(caps,domain_caps)
    if (not is_closed and float(initial_speed_mps)>float(caps[0])+1e-10) or (is_closed and float(initial_speed_mps)>float(caps[0])+1e-8):
        raise ValueError("initial speed exceeds valid capability-domain corner cap")
    # Keep the actual builder's endpoint/midpoint evaluation evidence. Invalid
    # results are immediately rejected by the builder; this counter is for
    # verification only and changes no capability/provider semantics.
    query_log=[]
    class _BuilderQueryAudit:
        def __init__(self, wrapped): self.wrapped=wrapped
        def __getattr__(self, name): return getattr(self.wrapped, name)
        def evaluate(self, query):
            result=self.wrapped.evaluate(query)
            row=(float(query.speed_mps),float(query.curvature_inv_m),float(query.soc),
                 bool(result.in_domain),tuple(result.limiting_constraints))
            query_log.append(row)
            if not result.in_domain:
                raise ValueError("hybrid builder query outside capability domain: "
                    f"speed_mps={row[0]:.12g}, curvature_inv_m={row[1]:.12g}, soc={row[2]:.12g}, "
                    f"constraints={','.join(row[4])}, builder_query_count={len(query_log)}")
            return result
    audited_provider=_BuilderQueryAudit(provider)
    plan=build_hybrid_speed_plan(x,k,caps,audited_provider,initial_speed_mps=float(initial_speed_mps),
        storage_soc=float(storage_soc),is_closed=bool(is_closed),brake_margin=float(brake_margin),
        yaw_moment_request_nm=0.,execution_allocator_identity=None)
    if not query_log or any(not row[3] for row in query_log):
        raise RuntimeError("hybrid builder query audit incomplete or out of domain")
    predictor=BrakeAuthorityPredictor(vehicle,maximum_abs_curvature_inv_m=max(abs(float(np.min(k))),abs(float(np.max(k)))),
        minimum_speed_mps=.005,maximum_speed_mps=float(speed_grid[-1]))
    config=HybridPolicyConfig(speed_gain=2.,throttle_acceleration_scale=float(throttle_acceleration_scale),
        maximum_brake_force_n=float(maximum_pedal_force_n),horizon_s=HYBRID_HORIZON_S,slabs=HYBRID_SLABS,
        pedal_context=pedal_context,brake_margin=float(brake_margin))
    policy=HybridLongitudinalPolicy(plan,provider,predictor,config,execution_allocator_identity=actual_allocator)
    actual_hashes=tuple(sorted((str(path),str(digest)) for path,digest in model_source_hashes.items()))
    source_closure_hash=_source_closure_hash(dict(actual_hashes))
    actual_arrays={
        "distance_m":x.tolist(),
        "curvature_inv_m":k.tolist(),
        "wheel_radii_m":list(radii),
        "physical_corner_caps_mps":physical_caps.tolist(),
        "capability_domain_speed_caps_mps":domain_caps.tolist(),
        "final_corner_caps_mps":caps.tolist(),
    }
    setup_payload={
        "schema":HYBRID_SETUP_VERSION,
        "setup_input_digest_schema":"sha256-canonical-json-v1",
        "mode":HYBRID_MODE,
        "source_file_hashes":actual_hashes,
        "source_closure_hash":source_closure_hash,
        "provider_identity":provider.identity,
        "model_hash":provider.model_hash,
        "calibration_hash":provider.calibration_hash,
        "grids":{"speed_mps":provider.speed_grid,"curvature_inv_m":provider.curvature_grid,"soc":provider.soc_grid},
        "inputs":{"maximum_speed_mps":float(maximum_speed_mps),"initial_speed_mps":float(initial_speed_mps),"storage_soc":soc,"maximum_pedal_force_n":float(maximum_pedal_force_n),"throttle_acceleration_scale":float(throttle_acceleration_scale),"lateral_margin":float(lateral_margin),"brake_margin":float(brake_margin),"yaw_moment_request_nm":float(yaw_moment_request_nm),"execution_allocator_identity":actual_allocator,"provider_identity":provider.identity,"is_closed":bool(is_closed)},
        "arrays":actual_arrays,
        "policy":{"version":HYBRID_POLICY_VERSION,"plan_version":HYBRID_PLAN_VERSION,"speed_gain":float(config.speed_gain),"throttle_acceleration_scale":float(config.throttle_acceleration_scale),"feedback_acceleration_limit_mps2":float(config.feedback_acceleration_limit_mps2),"maximum_brake_force_n":float(config.maximum_brake_force_n),"pedal_context":{name:getattr(pedal_context,name) for name in pedal_context.__dataclass_fields__},"governor_margin":float(config.governor_margin),"brake_margin":float(config.brake_margin),"horizon_s":float(config.horizon_s),"slabs":int(config.slabs),"max_extra_forecasts":HYBRID_POLICY_MAX_EXTRA_FORECASTS,"bracket_resolution":HYBRID_POLICY_BRACKET_RESOLUTION,"max_total_forecasts":HYBRID_POLICY_MAX_TOTAL_FORECASTS,"max_total_transitions":HYBRID_POLICY_MAX_TOTAL_FORECASTS*int(config.slabs)},
        "domain_cap_reserve_schema":HYBRID_DOMAIN_CAP_RESERVE_SCHEMA,
    }
    identity,identity_payload_json=_digest_payload(setup_payload)
    frozen_caps=_immutable_float_array(caps)
    frozen_physical=_immutable_float_array(physical_caps)
    frozen_domain=_immutable_float_array(domain_caps)
    return HybridSetup(provider,plan,policy,predictor,pedal_context,frozen_caps,frozen_physical,frozen_domain,identity,identity_payload_json,tuple(query_log))
