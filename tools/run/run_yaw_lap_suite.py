"""Bounded yaw lap studies with separate planning and saved-plan replay.

Always invoke through run_with_deadline.py (<=1800s). This entrypoint also
self-guards when launched directly, so test/planner cooperative budgets are not
its only wall-clock protection.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def parse_args(argv=None):
    from tools.run.yaw_map_options import add_yaw_map_options
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--track',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--mode',choices=('fixed','line','replay'),default='fixed')
    p.add_argument('--plan',type=Path)
    p.add_argument('--warm-plan',type=Path,help='validated coarser plan used only for speed-seed suggestions')
    p.add_argument('--spacing',type=float,default=2.)
    p.add_argument('--control-spacing',type=float,default=40.)
    p.add_argument('--profile-seconds',type=float,default=120.)
    p.add_argument('--profile-rounds',type=int,default=8)
    p.add_argument('--line-evaluations',type=int,default=24)
    p.add_argument('--line-seconds',type=float,default=900.)
    p.add_argument('--duration',type=float,default=150.)
    p.add_argument('--replay-seconds',type=float,default=300.)
    p.add_argument('--maximum-step',type=float,default=.05)
    p.add_argument('--controller',choices=('preview','model-aware'),default='preview')
    p.add_argument('--preview-seconds',type=float,default=None)
    p.add_argument('--heading-gain',type=float,default=None)
    p.add_argument('--lateral-gain',type=float,default=None)
    p.add_argument('--yaw-damping',type=float,default=None)
    add_yaw_map_options(p)
    args=p.parse_args(argv)
    if args.yaw_map is None:p.error('--yaw-map completed artifact is required; suite never rebuilds maps')
    if args.mode=='replay' and args.plan is None:p.error('replay requires --plan')
    return args


def run(args):
    from tools.run import run_yaw_aware_forward_driver as runner
    from tools.run.yaw_map_options import resolve_yaw_map
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.solvers.yaw_aware_profile import yaw_aware_profile
    from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings,YawProfileSolveError
    from simplified_vehicle_models.planning.yaw_lap_artifact import save_lap_plan,load_lap_plan
    output=runner.prepare_output(args.output_dir)
    vehicle=compile_formulation(str(runner.PLANAR_FORMULATION)).model
    pm=point_mass_view_of_planar(vehicle);vmax=runner.physical_map_speed_limit(pm)
    capability,map_report=resolve_yaw_map(pm,vehicle.yaw_inertia,np.linspace(0.,vmax,12),
        runner.anchored_axis(args.yaw_map_ax_min,args.yaw_map_ax_max,9),args)
    track=load_track(args.track)
    # Include transitive track/data bytes, not merely the XML pathname.
    from simplified_vehicle_models.formulations.yaw_map_artifact import source_provenance
    track_provenance=source_provenance(args.track)
    track_identity=hashlib.sha256(json.dumps(track_provenance,sort_keys=True).encode()).hexdigest()
    settings=CoupledProfileSettings(max_wall_seconds=args.profile_seconds,max_refinement_rounds=args.profile_rounds)
    from tools.run.yaw_study_contract import study_contract, finalized_plan_inputs, update_replay_initial_state
    try:
        import scipy, casadi
        packages={'scipy':scipy.__version__,'casadi':casadi.__version__}
    except ImportError as error:
        packages={'unavailable':str(error)}
    source_revision='unknown';dirty_hashes='unknown'
    try:
        import subprocess
        root=Path(__file__).resolve().parents[3]
        source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
        changed=subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True).splitlines()
        dirty_hashes={}
        for entry in changed:
            name=entry[3:]
            target=(root/name)
            if target.is_file():dirty_hashes[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    except (OSError,subprocess.SubprocessError):
        source_revision='unknown';dirty_hashes='unknown'
    provenance={'source_revision':source_revision,'source_dirty_hashes':dirty_hashes,
        'vehicle_configuration_hashes':source_provenance(str(runner.PLANAR_FORMULATION)), 
        'track_data_hashes':track_provenance,'map_file_sha256':hashlib.sha256(args.yaw_map.read_bytes()).hexdigest(),
        'map_source_identity':capability.source_identity,'plan_file_digest':'pending','plan_array_digest':'pending',
        # Planning has no forward-plant state; replay records its actual initial
        # state at invocation below. Unknown is never an attestation.
        'initial_state_soc':'unknown','initial_state':'unknown',
        'map_identity':capability.source_identity,'track_identity':track_identity,
        'maximum_speed_mps':vmax,'boundary_type':'flying' if track.is_closed else 'standing',
        'package_versions':packages,
        'thread_settings':{key:value for key,value in __import__('os').environ.items() if 'THREAD' in key.upper()},
        # The runner self-guards at 1800 s; callers may impose a shorter outer
        # watchdog which must be recorded by their run wrapper.
        'watchdog_budget_seconds':1800}
    controller_overrides={key:getattr(args,key) for key in ('preview_seconds','heading_gain','lateral_gain','yaw_damping')}
    warm_digest = (hashlib.sha256(args.warm_plan.read_bytes()).hexdigest()
                   if args.warm_plan is not None else 'none')
    resolved={'profile_settings':{key:getattr(settings,key) for key in settings.__dataclass_fields__},
        'controller':args.controller,'controller_overrides':controller_overrides,
        'warm_start_policy':'squared_speed_prolongation' if args.warm_plan is not None else 'cold',
        'warm_start_digest':warm_digest,
        'yaw_moment_tolerance':settings.yaw_moment_tolerance,
        'interval_q_tolerance':settings.interval_q_tolerance,'acceleration_tolerance':settings.acceleration_tolerance,
        'geometry_spacing_m':args.spacing,'control_spacing_m':args.control_spacing,
        'map_identity':capability.source_identity,'map_file_sha256':hashlib.sha256(args.yaw_map.read_bytes()).hexdigest(),
        'map_source_identity':capability.source_identity,'track_identity':track_identity,
        'track_data_hashes':track_provenance,'vehicle_configuration_hashes':source_provenance(str(runner.PLANAR_FORMULATION)),
        'boundary_type':'flying' if track.is_closed else 'standing','maximum_speed_mps':vmax,
        'replay_treatment':args.controller,
        'solver':{'method':'BDF','rtol':1e-6,'atol':1e-8,'maximum_step':args.maximum_step},
        'duration':args.duration,'maximum_step':args.maximum_step,'rtol':1e-6,'atol':1e-8,'method':'BDF',
        'profile_line_settings':{'spacing':args.spacing,'control_spacing':args.control_spacing,
            'profile_seconds':args.profile_seconds,'profile_rounds':args.profile_rounds,
            'line_evaluations':args.line_evaluations,'line_seconds':args.line_seconds}}
    contract=study_contract(vars(args),resolved,provenance)
    manifest={'args':vars(args),'map_identity':capability.source_identity,
        'track_identity':track_identity,'continuous_feasibility_certified':False,'study_contract':contract}
    runner.write_json(output/'inputs.json',manifest)
    try:
        if args.mode=='replay':
            path,speed_plan,arrays,metadata,validation=load_lap_plan(args.plan,capability,expected_track_identity=track_identity)
            finalized=finalized_plan_inputs(vars(args),resolved,provenance,args.plan,metadata['array_digest'],
                map_identity=capability.source_identity,track_identity=track_identity)
            runner.write_json(output/'inputs.json',finalized)
            return replay(args,output,vehicle,track,path,speed_plan,arrays,metadata)
        if args.mode=='fixed':
            from tools.run.probe_coupled_yaw_profile import _sample_geometry
            from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve,OffsetTrajectory
            from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
            reference=ReferenceCurve(track);spline=OffsetSpline(track,max(4.,args.spacing*100.))
            trajectory=OffsetTrajectory(reference,spline,np.zeros(spline.control_count))
            path=_sample_geometry(track,'zero-offset',args.spacing)
            source=trajectory.station_map.to_reference(path.distance)
            warm=None
            if args.warm_plan:
                _,_,warm_arrays,_,_=load_lap_plan(args.warm_plan,capability,expected_track_identity=track_identity)
                warm=np.sqrt(np.maximum(0.,np.interp(source,warm_arrays['reference_distance'],warm_arrays['speed']**2)))
                if path.is_closed:warm[-1]=warm[0]
            profile=yaw_aware_profile(path,capability,boundary='flying' if path.is_closed else 'standing',
                maximum_speed=vmax,solver='coupled',coupled_settings=settings,warm_start_speed=warm)
            geometry=trajectory.evaluate(source)
            geometry_data=dict(reference_distance=source,north=geometry.north,east=geometry.east,heading=geometry.heading)
            plan_diagnostics={'kind':'fixed_geometry','profile':dict(profile.diagnostics)}
        else:
            from simplified_vehicle_models.planning.yaw_aware_line import optimize_yaw_aware_line,YawLineSettings
            line=optimize_yaw_aware_line(track,None,capability,settings=YawLineSettings(
                control_spacing=args.control_spacing,grid_spacing=args.spacing,maximum_speed=vmax,
                profile_settings=settings,maximum_profile_evaluations=args.line_evaluations,
                maximum_wall_seconds=args.line_seconds))
            trajectory=line.trajectory;profile=line.profile
            from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
            source=trajectory.station_map.to_reference(profile.distance);geometry=trajectory.evaluate(source)
            from simplified_vehicle_models.solvers.yaw_aware_profile import curvature_rate_from_samples
            path=YawPath(profile.distance,geometry.curvature,curvature_rate_from_samples(profile.distance,
                geometry.curvature,is_closed=track.is_closed),track.is_closed)
            geometry_data=dict(reference_distance=source,north=geometry.north,east=geometry.east,heading=geometry.heading)
            plan_diagnostics={'kind':'optimized_line','status':line.status,'diagnostics':dict(line.diagnostics),
                              'coefficients':line.coefficients.tolist()}
        save_lap_plan(output/'plan.npz',yaw_path=path,speed=profile.speed,geometry=geometry_data,
            map_identity=capability.source_identity,track_identity=track_identity,maximum_speed=vmax,
            profile_settings=settings,provenance={**plan_diagnostics,'study_contract_version':contract['contract_version'],
                'study_contract_sha256':contract['canonical_json_sha256']})
        plan_path=output/'plan.npz'
        with np.load(plan_path,allow_pickle=False) as archive:
            saved_metadata=json.loads(str(archive['metadata'].item()))
        provenance.update(plan_file_digest=hashlib.sha256(plan_path.read_bytes()).hexdigest(),
                          plan_array_digest=saved_metadata['array_digest'])
        contract=study_contract(vars(args),resolved,provenance)
        runner.write_json(output/'inputs.json',{'args':vars(args),'map_identity':capability.source_identity,
            'track_identity':track_identity,'continuous_feasibility_certified':False,'study_contract':contract})
        # Load once through the strict replay gate before declaring a usable plan.
        _,_,_,saved_metadata,_=load_lap_plan(plan_path,capability,expected_track_identity=track_identity)
        summary={'plan_array_digest':saved_metadata['array_digest'],'status':'planned','lap_time_s':profile.lap_time,'speed_min_mps':float(min(profile.speed)),
            'speed_max_mps':float(max(profile.speed)),'profile_resolution_converged':profile.diagnostics.get('resolution_converged'),
            'profile_validation':profile.diagnostics.get('validation'),'diagnostics':plan_diagnostics,
            'lap_completed':False,'sampled_center_contained':None}
        runner.write_json(output/'summary.json',summary);return 0
    except (YawProfileSolveError,ValueError,RuntimeError,TimeoutError) as error:
        summary={'status':'failed','exception':type(error).__name__,'message':str(error),
            'cause':getattr(error,'status',None),'diagnostics':getattr(error,'diagnostics',None)}
        runner.write_json(output/'summary.json',summary);return 4


def replay(args,output,vehicle,track,path,speed_plan,arrays,metadata):
    from tools.run import run_yaw_aware_forward_driver as runner
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver,solve_forward
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
    from simplified_vehicle_models.tracks.types import PathCoordinates
    from simplified_vehicle_models.results.plant_channels import RunRecordSink
    from simplified_vehicle_models.tracks.corridor import corridor_from_track,SIGNED_POLICY
    from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
    if args.controller!='preview':
        from simplified_vehicle_models.solvers.yaw_tracking_driver import ModelAwareYawDriver
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        overrides={key:getattr(args,key) for key in ('preview_seconds','heading_gain','lateral_gain','yaw_damping')
                   if getattr(args,key) is not None}
        driver=ModelAwareYawDriver.for_vehicle(speed_plan,vehicle,point_mass_view_of_planar(vehicle),**overrides)
    else:
        driver=PreviewDriver(speed_plan,vehicle.wheelbase,vehicle.steering_maximum_angle,
            vehicle.steering_maximum_rate,vehicle.maximum_pedal_force)
    geometry=PathCoordinates(path.distance,arrays['north'],arrays['east'],arrays['heading'],path.curvature,
        is_closed=path.is_closed,reference_distance=arrays['reference_distance'])
    plant=PlanarForwardPlant(vehicle,speed_plan)
    sink=RunRecordSink(plant,geometry,output/'run_record.npz',precision='float64')
    try:
        initial_state=plant.initial_state(float(speed_plan.speed[0]))
        update_replay_initial_state(output/'inputs.json',initial_state,plant.state_names)
        result=solve_forward(plant,driver,initial_state,duration=args.duration,
            wall_seconds=args.replay_seconds,maximum_wall_seconds=min(1500.,args.replay_seconds),
            maximum_step=args.maximum_step,rtol=1e-6,atol=1e-8,method='BDF',step_sink=sink)
    except (RuntimeError,TimeoutError,ValueError) as error:
        if hasattr(error,'last_state'):
            runner.write_json(output/'integration_failure.json',{'diagnostics':getattr(error,'diagnostics',{}),
                'last_solver_time':error.last_time,'last_solver_state':error.last_state,
                'snapshot_validated':False,'state_names':plant.state_names})
        try:
            sink.close({'model':{'yaw_profile':metadata,'controller':args.controller},
                        'segment':{'lap_completed':False,'exception':str(error)}})
        except RuntimeError:
            pass
        raise
    sink.close({'model':{'yaw_profile':metadata,'controller':args.controller},
                'segment':{'lap_completed':bool(result.lap_completed)}})
    np.savetxt(output/'trace.csv',np.column_stack((result.time,result.state.T)),delimiter=',',comments='',
        header=','.join(('time',*plant.state_names)))
    observations=[plant.observe(row) for row in result.state.T]
    # Convert physical selected-path station back to source station, then add
    # actual vehicle lateral tracking error to the selected line offset.
    reference=ReferenceCurve(track)
    source=np.interp(np.array([o.distance for o in observations])%path.distance[-1],
        path.distance,arrays['reference_distance'])
    # Project selected path displacement onto reference right normal.
    refs=reference.evaluate(arrays['reference_distance'])
    line_offset=(arrays['north']-refs.north)*(-np.sin(refs.heading))+(arrays['east']-refs.east)*np.cos(refs.heading)
    center=np.interp(source,arrays['reference_distance'],line_offset)+np.array([o.lateral_error for o in observations])
    corridor=corridor_from_track(track,margin=0.,policy=SIGNED_POLICY,reference=reference)
    lo=np.interp(source,corridor.distance,corridor.minimum);hi=np.interp(source,corridor.distance,corridor.maximum)
    violation=np.maximum(np.maximum(lo-center,center-hi),0.)
    dt=np.diff(result.time);duration=max(float(np.sum(dt)),1e-12)
    lateral=np.array([o.lateral_error for o in observations]);speed=np.array([o.speed for o in observations])
    target=np.array([speed_plan.sample(o.distance)[0] for o in observations])
    rms=lambda values:float(np.sqrt(np.sum(dt*(values[:-1]**2+values[1:]**2)*.5)/duration))
    summary={'status':'completed' if result.lap_completed else 'incomplete','lap_completed':bool(result.lap_completed),
        'controller':args.controller,'driven_time_s':float(result.time[-1]),'lateral_error_rms_m':rms(lateral),
        'lateral_error_max_m':float(np.max(np.abs(lateral))),'speed_error_rms_mps':rms(speed-target),
        'sampled_center_corridor_violation_m':float(max(violation)),'sampled_center_contained':bool(max(violation)<=1e-8),
        'footprint_containment_certified':False,'between_step_containment_certified':False}
    controls=np.array([driver.command(observation) for observation in observations],dtype=object)
    throttle=np.array([command.throttle for command in controls]);brake=np.array([command.brake_force for command in controls])
    steering_rate=np.array([command.steering_rate for command in controls])
    weighted_fraction=lambda flags:float(np.sum(dt*(flags[:-1].astype(float)+flags[1:].astype(float))*.5)/duration)
    summary.update(throttle_max_requested=float(max(throttle)),
        throttle_saturation_time_fraction_requested=weighted_fraction(throttle>=1.-1e-6),
        brake_saturation_time_fraction_requested=weighted_fraction(brake>=vehicle.maximum_pedal_force*(1.-1e-6)),
        steering_rate_saturation_time_fraction_requested=weighted_fraction(abs(steering_rate)>=vehicle.steering_maximum_rate*(1.-1e-6)),
        control_metrics_kind='requested_commands; saved run record contains realized pedal-limited controls')
    runner.write_json(output/'summary.json',summary)
    return 0 if result.lap_completed else 3


def main(argv=None):
    return run(parse_args(argv))


if __name__=='__main__':
    from tools.run.run_with_deadline import run_with_deadline
    import os
    if os.environ.get('YAW_PROCESS_WATCHDOG_ACTIVE')=='1':
        # Stay in the existing guarded process group: starting another session
        # here would let a nested child escape the shorter outer hard deadline.
        raise SystemExit(main())
    raise SystemExit(run_with_deadline([sys.executable,__file__,*sys.argv[1:]],seconds=1800))
