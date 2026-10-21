"""Minimum-time quasi-steady racing line in the distance domain.

Decision vector is grouped: [n | psi | v | a | dpsi/ds | a_mid | dpsi/ds_mid].
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class QssSettings:
    """Transcription and solver settings."""
    fold_guard: float = 0.2
    maximum_heading: float = 0.6
    maximum_curvature_rate: float = 0.5
    minimum_speed: float = 0.5
    regularisation: float = 1.0e-6
    tolerance: float = 1.0e-6
    maximum_iterations: int = 3000


@dataclass(frozen=True, slots=True)
class QssSolution:
    """One accepted transcription solution."""
    distance: FloatArray
    offset: FloatArray
    heading_error: FloatArray
    speed: FloatArray
    acceleration: FloatArray
    curvature_rate: FloatArray
    lap_time: float
    status: str
    iterations: int
    maximum_violation: float


def build_mesh(track, spacing: float) -> FloatArray:
    """Uniform mesh ending exactly at lap length."""
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be finite and positive")
    length = float(track.lap_length)
    count = max(int(math.ceil(length / spacing)), 4)
    return np.linspace(0.0, length, count + 1)


def periodic_sample(values, distance, positions) -> FloatArray:
    """Linearly sample periodic track data."""
    values = np.asarray(values, dtype=float)
    distance = np.asarray(distance, dtype=float)
    query = np.asarray(positions, dtype=float)
    if values.ndim != 1 or distance.shape != values.shape or values.size < 2:
        raise ValueError("periodic data must be matching one-dimensional arrays")
    period = distance[-1] - distance[0]
    extended_nodes = np.r_[distance, distance[-1] + (distance[1] - distance[0])]
    extended_values = np.r_[values, values[0]]
    wrapped = (query - distance[0]) % period + distance[0]
    wrapped = np.where(np.isclose(query, distance[-1]), distance[-1], wrapped)
    return np.interp(wrapped, extended_nodes, extended_values)


def _element_curvature(track, mesh) -> FloatArray:
    return periodic_sample(track.curvature, track.distance, mesh)


def build_ocp(track, corridor, surface, mesh, settings: QssSettings):
    """Build CasADi NLP and its variable/constraint bounds."""
    import casadi as ca

    mesh = np.asarray(mesh, dtype=float)
    if not getattr(track, "is_closed", False):
        raise ValueError("quasi-steady line OCP requires a closed track")
    if not getattr(corridor, "is_periodic", False):
        raise ValueError("quasi-steady line OCP requires a periodic corridor")
    if mesh.ndim != 1 or mesh.size < 2 or np.any(np.diff(mesh) <= 0):
        raise ValueError("mesh must be a strictly increasing vector")
    count, nodes = mesh.size - 1, mesh.size
    amax = ca.interpolant("qss_amax", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_max.ravel(order="F").tolist())
    amin = ca.interpolant("qss_amin", "linear", [surface.speed.tolist(), surface.curvature.tolist()], surface.acceleration_min.ravel(order="F").tolist())
    ceiling = ca.interpolant("qss_vceil", "linear", [surface.curvature.tolist()], surface.speed_ceiling.tolist())
    k0 = _element_curvature(track, mesh)
    nlo = periodic_sample(corridor.minimum, corridor.distance, mesh)
    nhi = periodic_sample(corridor.maximum, corridor.distance, mesh)
    n = [ca.MX.sym(f"n_{i}") for i in range(nodes)]
    psi = [ca.MX.sym(f"psi_{i}") for i in range(nodes)]
    v = [ca.MX.sym(f"v_{i}") for i in range(nodes)]
    a = [ca.MX.sym(f"a_{i}") for i in range(nodes)]
    q = [ca.MX.sym(f"q_{i}") for i in range(nodes)]
    am = [ca.MX.sym(f"am_{i}") for i in range(count)]
    qm = [ca.MX.sym(f"qm_{i}") for i in range(count)]
    x = ca.vertcat(*n, *psi, *v, *a, *q, *am, *qm)
    abox = float(max(np.max(surface.acceleration_max), -np.min(surface.acceleration_min)))
    xl = np.concatenate([nlo, np.full(nodes,-settings.maximum_heading), np.full(nodes,settings.minimum_speed), np.full(nodes,-abox), np.full(nodes,-settings.maximum_curvature_rate), np.full(count,-abox), np.full(count,-settings.maximum_curvature_rate)])
    xu = np.concatenate([nhi, np.full(nodes,settings.maximum_heading), np.full(nodes,float(surface.speed[-1])), np.full(nodes,abox), np.full(nodes,settings.maximum_curvature_rate), np.full(count,abox), np.full(count,settings.maximum_curvature_rate)])

    def rates(state, control, curvature):
        offset, heading, speed = state
        acceleration, rate = control
        den = 1 - curvature * offset
        return ca.vertcat(ca.tan(heading)*den, rate, acceleration*den/(speed*ca.cos(heading)))

    def kap(offset, heading, rate, curvature):
        return (curvature + rate)*ca.cos(heading)/(1-curvature*offset)

    constraints, gl, gu = [], [], []
    def add(expr, low, high):
        constraints.append(expr); gl.append(low); gu.append(high)
    def limits(offset, heading, speed, accel, rate, curvature):
        vehicle_k = kap(offset, heading, rate, curvature)
        add(vehicle_k, float(surface.curvature[0]), float(surface.curvature[-1]))
        add(accel-amin(ca.vertcat(speed,vehicle_k)),0.0,float("inf"))
        add(amax(ca.vertcat(speed,vehicle_k))-accel,0.0,float("inf"))
        add(ceiling(vehicle_k)-speed,0.0,float("inf"))
        add(1-curvature*offset,settings.fold_guard,float("inf"))

    objective = settings.regularisation*ca.sumsqr(ca.vertcat(*q))
    for i in range(count):
        h = float(mesh[i+1]-mesh[i]); midpos=.5*(mesh[i]+mesh[i+1])
        kmid=float(_element_curvature(track,[midpos])[0])
        s0=ca.vertcat(n[i],psi[i],v[i]); s1=ca.vertcat(n[i+1],psi[i+1],v[i+1])
        f0=rates((n[i],psi[i],v[i]),(a[i],q[i]),float(k0[i]))
        f1=rates((n[i+1],psi[i+1],v[i+1]),(a[i+1],q[i+1]),float(k0[i+1]))
        sm=.5*(s0+s1)+h/8*(f0-f1)
        fm=rates((sm[0],sm[1],sm[2]),(am[i],qm[i]),kmid)
        defect=s1-s0-h/6*(f0+4*fm+f1)
        for row in range(3): add(defect[row],0.0,0.0)
        def time_rate(offset,heading,speed,curvature):
            return (1-curvature*offset)/(speed*ca.cos(heading))
        objective += h/6*(time_rate(n[i],psi[i],v[i],float(k0[i]))+4*time_rate(sm[0],sm[1],sm[2],kmid)+time_rate(n[i+1],psi[i+1],v[i+1],float(k0[i+1])))
        limits(n[i],psi[i],v[i],a[i],q[i],float(k0[i]))
        limits(sm[0],sm[1],sm[2],am[i],qm[i],kmid)
        limits(n[i+1],psi[i+1],v[i+1],a[i+1],q[i+1],float(k0[i+1]))
    add(n[0]-n[-1],0.0,0.0); add(psi[0]-psi[-1],0.0,0.0); add(v[0]-v[-1],0.0,0.0)
    return {"nlp":{"x":x,"f":objective,"g":ca.vertcat(*constraints)},"x_lower":xl,"x_upper":xu,"g_lower":np.asarray(gl),"g_upper":np.asarray(gu),"mesh":mesh,"count":count}


def unpack(values, count: int) -> dict[str, FloatArray]:
    nodes=count+1; expected=5*nodes+2*count; flat=np.asarray(values,dtype=float).ravel()
    if flat.size != expected: raise ValueError(f"decision vector has {flat.size} entries, layout needs {expected}")
    names=("offset","heading","speed","acceleration","curvature_rate")
    blocks={name:flat[i*nodes:(i+1)*nodes] for i,name in enumerate(names)}
    tail=flat[5*nodes:]; blocks["mid_acceleration"]=tail[:count]; blocks["mid_curvature_rate"]=tail[count:]
    return blocks


def initial_guess(track, corridor, surface, mesh, settings: QssSettings) -> FloatArray:
    mesh=np.asarray(mesh,dtype=float); count=mesh.size-1
    offset=.5*(periodic_sample(corridor.minimum,corridor.distance,mesh)+periodic_sample(corridor.maximum,corridor.distance,mesh))
    curvature=np.clip(_element_curvature(track,mesh),surface.curvature[0],surface.curvature[-1])
    ceiling=np.interp(curvature,surface.curvature,surface.speed_ceiling)
    speed=np.full(mesh.size,max(.6*float(np.min(ceiling)),settings.minimum_speed+.1))
    zeros=np.zeros(mesh.size); mids=np.zeros(count)
    return np.concatenate([offset,zeros,speed,zeros,zeros,mids,mids])


def solve_line(track, corridor, surface, mesh, settings: QssSettings, *, seed: FloatArray | None = None) -> QssSolution:
    """Solve the free-line minimum-time problem on a fixed mesh."""
    import casadi as ca
    built=build_ocp(track,corridor,surface,mesh,settings); count=built["count"]
    vector=initial_guess(track,corridor,surface,mesh,settings) if seed is None else np.asarray(seed,dtype=float).ravel()
    if vector.size != built["nlp"]["x"].shape[0]: raise ValueError("seed does not match the mesh")
    vector=np.clip(vector,built["x_lower"],built["x_upper"])
    opts={"ipopt.tol":settings.tolerance,"ipopt.constr_viol_tol":settings.tolerance,"ipopt.max_iter":settings.maximum_iterations,"ipopt.hessian_approximation":"exact","ipopt.nlp_scaling_method":"gradient-based","ipopt.print_level":0,"print_time":False}
    solver=ca.nlpsol("qss_line","ipopt",built["nlp"],opts)
    result=solver(x0=vector,lbx=built["x_lower"],ubx=built["x_upper"],lbg=built["g_lower"],ubg=built["g_upper"])
    values=np.asarray(result["x"],dtype=float).ravel(); g=np.asarray(result["g"],dtype=float).ravel()
    violation=float(np.max(np.maximum(np.maximum(built["g_lower"]-g,0),g-built["g_upper"])))
    stats=solver.stats(); blocks=unpack(values,count)
    return QssSolution(np.asarray(mesh,dtype=float),blocks["offset"],blocks["heading"],blocks["speed"],blocks["acceleration"],blocks["curvature_rate"],float(result["f"]),str(stats["return_status"]),int(stats["iter_count"]),violation)


def project_seed(coarse: QssSolution, mesh) -> FloatArray:
    """Interpolate a prior solution to the requested mesh."""
    mesh=np.asarray(mesh,dtype=float); count=mesh.size-1; source=np.asarray(coarse.distance,dtype=float)
    if source.size != coarse.offset.size: raise ValueError("solution distance does not match its node count")
    def periodic(values): return np.r_[np.asarray(values,dtype=float),np.asarray(values,dtype=float)[0]]
    arrays=[np.interp(mesh,source,periodic(values)) for values in (coarse.offset,coarse.heading_error,coarse.speed,coarse.acceleration,coarse.curvature_rate)]
    mids=np.zeros(count)
    return np.concatenate([*arrays,mids,mids])
