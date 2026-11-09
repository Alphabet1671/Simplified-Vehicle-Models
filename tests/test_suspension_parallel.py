import unittest
from types import SimpleNamespace
from pathlib import Path
import shutil
import tempfile

import numpy as np

from simplified_vehicle_models.suspension.damper_table import DamperForce
from simplified_vehicle_models.suspension.layouts.parallel import (
    ROWS, STRUT_NAMES, ParallelNetwork, build_parallel_suspension,
)
from simplified_vehicle_models.suspension.physical import LinearDamper, PhysicalStrut
from simplified_vehicle_models.compiler.compile import compile_formulation, compile_component
from simplified_vehicle_models.compiler.registry import ComponentCandidate, component_descriptor
from simplified_vehicle_models.xml.loader import load_model_shallow
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.suspension.physical_components import (
    compile_linear_damper, compile_physical_strut,
)


class Kinematics:
    static_sprung_load = np.array([5., 5., 7., 7.])
    jounce_limits = np.tile([-1., 1.], (4, 1))

    def motion_ratio(self, wheel, jounce):
        return (1.05, 1.05, 1.10, 1.10)[wheel]


def struts():
    return tuple(PhysicalStrut(k, p, b, LinearDamper(c), scale, mr, orient)
                 for k, p, b, c, scale, mr, orient in [
        (100., 1., 2., 3., 1., 1., 1.), (110., 2., 3., 4., 1., 1., 1.),
        (120., 3., 4., 5., 1., 1.1, -1.), (130., 4., 5., 6., 1., 1.1, 1.),
        (20., 0., 6., 7., 1., 1.05, 1.), (30., 0., 7., 8., 1., 1.2, 1.),
        (40., 0., 8., 9., 1., 1.1, 1.), (50., 0., 9., 10., 1., 1.3, 1.)])


class AsymmetricDamper:
    """Signed, asymmetric compression/rebound force law with flagged outer range."""
    def force(self, velocity):
        v = np.asarray(velocity, dtype=float)
        force = np.where(v >= 0., 2. * v, 5. * v)
        flag = np.abs(v) > .05
        return DamperForce(force, np.where(v >= 0., 2., 5.), flag)


def _model(model_type, subtype, parameters, units, refs=(), data=(), path='fixture.xml'):
    return Model(path=Path(path), model_id='fixture', model_type=model_type, subtype=subtype,
                 parameters=parameters, units=units, references=tuple(refs), data=tuple(data))


class PhysicalCompilerTests(unittest.TestCase):
    def test_linear_damper_zero_and_closed_units(self):
        zero = compile_linear_damper(_model('damper-model','linear', {'coefficient':0.}, {'coefficient':'N*s/m'}))
        np.testing.assert_array_equal(zero.force([1., -1.]).force, [0., 0.])
        with self.assertRaisesRegex(ValueError, 'unit'):
            compile_linear_damper(_model('damper-model','linear', {'coefficient':0.}, {'coefficient':'N/m'}))
        with self.assertRaisesRegex(ValueError, 'unknown'):
            compile_linear_damper(_model('damper-model','linear', {'coefficient':0., 'extra':1.}, {'coefficient':'N*s/m','extra':'1'}))
        with self.assertRaisesRegex(ValueError, 'finite'):
            compile_linear_damper(_model('damper-model','linear', {'coefficient':float('nan')}, {'coefficient':'N*s/m'}))

    def test_strut_compiles_table_mode_or_linear_child_and_validates_values(self):
        child = LinearDamper(4.)
        model = _model('suspension-strut','physical',
            dict(spring_rate=10., preload=2., inertance=0., damper_scale=1., motion_ratio=1.2, orientation=-1.),
            dict(spring_rate='N/m', preload='N', inertance='kg', damper_scale='1', motion_ratio='1', orientation='1'))
        self.assertIs(compile_physical_strut(model, {'damper':child}).damper, child)
        table_child = SimpleNamespace(force=lambda velocity: None)
        self.assertIs(compile_physical_strut(model, {'damper':table_child}).damper, table_child)
        for values, message in (
            (dict(spring_rate=0.,preload=1.,inertance=0.,damper_scale=0.,motion_ratio=1.,orientation=1.), 'preload'),
            (dict(spring_rate=1.,preload=0.,inertance=0.,damper_scale=0.,motion_ratio=1.,orientation=0.), 'orientation'),
            (dict(spring_rate=10.,preload=0.,inertance=0.,damper_scale=1.,motion_ratio=0.,orientation=1.), 'motion_ratio'),
            (dict(spring_rate=float('nan'),preload=0.,inertance=0.,damper_scale=0.,motion_ratio=1.,orientation=1.), 'finite'),
        ):
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                compile_physical_strut(_model('suspension-strut','physical', values,
                    dict(spring_rate='N/m',preload='N',inertance='kg',damper_scale='1',motion_ratio='1',orientation='1')),
                    {'damper':child})
        with self.assertRaisesRegex(ValueError, 'unit'):
            compile_physical_strut(_model('suspension-strut','physical',
                dict(spring_rate=1.,preload=0.,inertance=0.,damper_scale=0.,motion_ratio=1.,orientation=1.),
                dict(spring_rate='N',preload='N',inertance='kg',damper_scale='1',motion_ratio='1',orientation='1')), {'damper':child})
        self.assertEqual(component_descriptor('damper-model','linear').subtype, 'linear')
        self.assertEqual(component_descriptor('suspension-assembly','parallel-corner-axle').child_roles,
                         ('kinematics', *STRUT_NAMES))

    def test_real_temporary_xml_parser_registry_nested_compilation_and_failures(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'models'
            shutil.copytree(Path('models').resolve(), root, dirs_exist_ok=True)
            assembly_path = root / 'suspensions/iem24_murphy_real_suspension.xml'
            text = assembly_path.read_text().replace(
                'role="corner-fl" ref="../struts/iem24_corner_zero_strut.xml"',
                'role="corner-fl" ref="../struts/iem24_corner_mixed_strut.xml"')
            assembly_path.write_text(text)
            mixed = root / 'struts/iem24_corner_mixed_strut.xml'
            mixed.write_text((root / 'struts/iem24_corner_zero_strut.xml').read_text().replace(
                'id="iem24-corner-zero-strut"', 'id="iem24-corner-mixed-strut"').replace(
                'name="spring_rate" value="0"', 'name="spring_rate" value="100"'))
            assembly = load_model_shallow(assembly_path)
            descriptor = component_descriptor(assembly.model_type, assembly.subtype)
            def compile_tree():
                return compile_component(ComponentCandidate('fixture', load_model_shallow(assembly_path), descriptor))
            loaded = compile_tree()
            self.assertEqual(loaded.value.layout, 'parallel-corner-axle')
            self.assertGreater(loaded.value.network.struts[0].spring_rate, 0.)
            self.assertGreater(loaded.value.network.struts[4].spring_rate, 0.)

            missing = assembly_path.read_text().replace(
                '<component role="roll-rear" ref="../struts/iem24_roll_rear_strut.xml"/>', '')
            assembly_path.write_text(missing)
            with self.assertRaisesRegex(ValueError, 'missing child role'):
                compile_tree()
            assembly_path.write_text(text)

            heave_path = root / 'struts/iem24_heave_front_strut.xml'
            original = heave_path.read_text()
            cases = (
                (original.replace('unit="N/m"', 'unit="N"', 1), 'unit'),
                (original.replace('</parameters>', '<parameter name="surprise" value="1" unit="1"/></parameters>'), 'unknown'),
                (original.replace('value="1.05"', 'value="0"'), 'motion_ratio'),
                (original.replace('name="orientation" value="1"', 'name="orientation" value="0"'), 'orientation'),
                (original.replace('value="1062.133744875"', 'value="1"'), 'do not match static sprung loads'),
                (original.replace('value="61294.39233626669"', 'value="NaN"'), 'finite'),
            )
            for xml, message in cases:
                with self.subTest(message=message):
                    heave_path.write_text(xml)
                    with self.assertRaisesRegex(ValueError, message):
                        compile_tree()
            heave_path.write_text(original)

    def test_active_murphy_is_additive_physical_configuration(self):
        compiled = compile_formulation('models/formulations/iem24_murphy_real_14dof_simple.xml')
        network = compiled.model.suspension.network
        self.assertEqual(compiled.model.suspension.layout, 'parallel-corner-axle')
        self.assertEqual(network.n_states, 0)
        self.assertFalse(network.has_inertia)
        for s in network.struts[:4]:
            self.assertEqual((s.spring_rate,s.preload,s.inertance,s.damper_scale),(0.,0.,0.,0.))
        conversion = 175.12683524647627
        np.testing.assert_allclose([s.spring_rate for s in network.struts[4:]], np.array([350.,225.,350.,225.])*conversion)
        np.testing.assert_allclose(network.static_check(), compiled.model.suspension.kinematics.static_sprung_load)
        self.assertEqual([network.struts[i].damper.setting.clicks for i in range(4,8)],
                         [{'compression':6.,'rebound':6.},{'compression':5.,'rebound':5.},
                          {'compression':6.,'rebound':6.},{'compression':7.,'rebound':7.}])
        heave = network.struts[4].damper.table.metadata['file_sha256']
        self.assertEqual(network.struts[6].damper.table.metadata['file_sha256'], heave)
        roll = network.struts[5].damper.table.metadata['file_sha256']
        self.assertEqual(network.struts[7].damper.table.metadata['file_sha256'], roll)
        self.assertNotEqual(heave, roll)
        self.assertEqual([s.motion_ratio for s in network.struts[4:]], [1.05,1.20,1.10,1.30])
        self.assertEqual(compiled.manifest.formulation.model_id, 'iem24-murphy-real-14dof-simple')
        self.assertEqual(compiled.manifest.formulation.path.resolve(), Path('models/formulations/iem24_murphy_real_14dof_simple.xml').resolve())
        self.assertEqual(compiled.manifest.assembly.model_id, 'iem24-murphy-real-planar')
        self.assertEqual(compiled.manifest.assembly.path.resolve(), Path('models/vehicles/iem24_murphy_real_planar.xml').resolve())
        self.assertEqual(network.struts[5].orientation, 1.)
        roles = {record.role for record in compiled.manifest.loaded_components}
        self.assertIn('suspension/corner-fl', roles)
        self.assertIn('suspension/heave-front', roles)
        self.assertIn('suspension/roll-rear', roles)
        self.assertNotIn('suspension/arb', roles)
        self.assertNotIn('arb', roles)
        self.assertEqual(compiled.manifest.assembly.model_id, 'iem24-murphy-real-planar')
        suspension_xml = Path('models/suspensions/iem24_murphy_real_suspension.xml').read_text()
        self.assertNotIn('role="arb"', suspension_xml)
        np.testing.assert_allclose([network.struts[4].preload, network.struts[6].preload],
                                   [2.*1.05*505.77797375, 2.*1.10*646.50340125], rtol=0., atol=1e-12)
        self.assertEqual(network.struts[5].preload, 0.)
        self.assertEqual(network.struts[7].preload, 0.)
        self.assertGreater(network.forces([0,0,0,0],[-.01,.01,0,0]).element_velocity['roll-front'],0.)
        self.assertGreater(network.forces([0,0,0,0],[0,0,-.01,.01]).element_velocity['roll-rear'],0.)

    def test_motion_ratio_is_single_wheel_travel_over_unit_end_to_end_travel(self):
        """Pin the motion ratio definition of the active Murphy struts.

        MR = (travel of one wheel in the strut's mode) / (end-to-end travel of the
        real spring/damper unit). Pure heave moves both wheels of an axle by z;
        pure roll moves one wheel by +z and the other by -z. In both cases the
        unit length changes by z / MR, and its velocity by zdot / MR. A single
        wheel bump of z is half heave plus half roll, so each axle unit then sees
        z / (2 * MR).
        """
        network = compile_formulation('models/formulations/iem24_murphy_real_14dof_simple.xml').model.suspension.network
        z, zdot = 0.010, 0.20
        modes = {  # strut index: (unit-mode wheel travel pattern, MR from XML)
            4: (np.array([z, z, 0., 0.]), 1.05),    # heave-front
            5: (np.array([-z, z, 0., 0.]), 1.20),   # roll-front: left rebound, right bump
            6: (np.array([0., 0., z, z]), 1.10),    # heave-rear
            7: (np.array([0., 0., -z, z]), 1.30),   # roll-rear
        }
        for index, (wheels, ratio) in modes.items():
            with self.subTest(strut=STRUT_NAMES[index]):
                self.assertEqual(network.struts[index].motion_ratio, ratio)
                result = network.forces(wheels, wheels / z * zdot)
                travel = (network.jacobian @ wheels)[index]
                velocity = result.element_velocity[STRUT_NAMES[index]]
                # One wheel's travel over the unit travel is the declared MR.
                self.assertAlmostEqual(z / travel, ratio, places=12)
                self.assertAlmostEqual(zdot / velocity, ratio, places=12)
                # The other axle-mode units do not move.
                others = [i for i in range(4, 8) if i != index]
                np.testing.assert_allclose((network.jacobian @ wheels)[others], 0., atol=1e-15)
        # A single-wheel bump is half heave plus half roll.
        single = network.jacobian @ np.array([z, 0., 0., 0.])
        self.assertAlmostEqual(single[4], z / (2. * 1.05), places=15)
        self.assertAlmostEqual(single[5], -z / (2. * 1.20), places=15)
        # Each wheel sees k / (2 MR^2) in its own mode, because two wheels share one unit.
        epsilon = 1e-6
        zero = network.static_forces(np.zeros(4))
        heave_rate = (network.static_forces(np.array([epsilon, epsilon, 0., 0.])) - zero)[0] / epsilon
        roll_rate = (network.static_forces(np.array([-epsilon, epsilon, 0., 0.])) - zero)[1] / epsilon
        self.assertAlmostEqual(heave_rate, network.struts[4].spring_rate / (2. * 1.05**2), delta=1e-6 * heave_rate)
        self.assertAlmostEqual(roll_rate, network.struts[5].spring_rate / (2. * 1.20**2), delta=1e-6 * roll_rate)

    def test_actual_murphy_network_allows_far_finite_jounce(self):
        network = compile_formulation('models/formulations/iem24_murphy_real_14dof_simple.xml').model.suspension.network
        z = np.array([-.1, .1, .1, -.1])
        force = network.forces(z, np.zeros(4)).total
        static = network.static_forces(z)
        A = network.jacobian
        x = A @ z
        expected_shaft = np.array([s.preload + s.spring_rate * x[i]
                                   for i, s in enumerate(network.struts)])
        expected_force = A.T @ expected_shaft
        np.testing.assert_allclose(force, expected_force, rtol=0., atol=1e-10)
        np.testing.assert_allclose(static, expected_force, rtol=0., atol=1e-10)
        np.testing.assert_allclose(network.inertia_block(z),
                                   A.T @ (np.array([s.inertance for s in network.struts])[:, None] * A),
                                   rtol=0., atol=1e-12)


class ParallelNetworkTests(unittest.TestCase):
    def setUp(self):
        self.kin = Kinematics()
        self.struts = struts()
        self.network = ParallelNetwork(self.kin, self.struts)

    def oracle(self,z,zdot):
        A=ROWS*np.array([s.orientation/s.motion_ratio for s in self.struts])[:,None]; x,v=A@z,A@zdot
        return A,x,v,np.array([s.preload for s in self.struts])+np.array([s.spring_rate for s in self.struts])*x+np.array([s.damper.coefficient*s.damper_scale for s in self.struts])*v,np.array([s.inertance for s in self.struts])

    def test_full_force_inertia_power_and_diagnostic_sign(self):
        z=np.array([.01,.02,-.01,.03]); zdot=np.array([-.01,.01,-.01,.01]); A,x,v,shaft,b=self.oracle(z,zdot); result=self.network.forces(z,zdot)
        np.testing.assert_allclose(result.total,A.T@shaft); np.testing.assert_allclose(self.network.inertia_block(z),A.T@(b[:,None]*A)); self.assertAlmostEqual(float(zdot@result.total),float(v@shaft)); self.assertGreater(result.element_velocity['roll-front'],0.); self.assertLess(self.network.forces(z,-zdot).element_velocity['roll-front'],0.)

    def test_zero_devices_and_static_spring_path(self):
        zero=tuple(PhysicalStrut(0.,0.,0.,LinearDamper(0.),0.,1.,1.) for _ in range(8)); net=ParallelNetwork(self.kin,zero)
        np.testing.assert_array_equal(net.forces(np.zeros(4),np.ones(4)).total,np.zeros(4)); np.testing.assert_array_equal(net.inertia_block(np.zeros(4)),np.zeros((4,4))); np.testing.assert_array_equal(net.static_forces(np.zeros(4)),np.zeros(4))

    def test_elements_coexist_without_double_preload_and_axles_decouple(self):
        z=np.array([.012,-.007,.018,-.011])
        v=np.array([.14,-.09,.11,-.16])
        result=self.network.forces(z,v)
        A,_,_,shaft,_=self.oracle(z,v)
        np.testing.assert_allclose(result.total,A.T@shaft)
        rear_changed=list(self.struts)
        rear_changed[6]=PhysicalStrut(400.,0.,8.,LinearDamper(9.),1.,1.1,1.)
        changed=ParallelNetwork(self.kin,tuple(rear_changed)).forces(z,v)
        np.testing.assert_allclose(changed.total[:2],result.total[:2])
        self.assertGreater(np.linalg.norm(changed.total[2:] - result.total[2:]), 0.)
        np.testing.assert_allclose(result.elements['preload'],A.T@np.array([s.preload for s in self.struts]))

    def test_modal_matrix_properties_and_constant_ratio_forcing(self):
        mass=self.network.inertia_block(np.zeros(4))
        np.testing.assert_allclose(mass,mass.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(mass).min(),-1e-12)
        modal=list(self.struts)
        for i,s in enumerate(modal):
            modal[i]=PhysicalStrut(s.spring_rate,s.preload,0.,s.damper,s.damper_scale,s.motion_ratio,s.orientation)
        modal[4]=PhysicalStrut(0.,0.,3.,LinearDamper(0.),0.,1.,1.)
        heave_mass=ParallelNetwork(self.kin,tuple(modal)).inertia_block(np.zeros(4))
        self.assertGreater(heave_mass[0,1],0.)
        modal[4]=PhysicalStrut(0.,0.,0.,LinearDamper(0.),0.,1.,1.)
        modal[5]=PhysicalStrut(0.,0.,1.,LinearDamper(0.),0.,1.,1.)
        roll_mass=ParallelNetwork(self.kin,tuple(modal)).inertia_block(np.zeros(4))
        self.assertLess(roll_mass[0,1],0.)
        self.assertEqual(self.network.inertial_forcing(np.zeros(4),np.ones(4)).shape,(4,))
        np.testing.assert_array_equal(self.network.inertial_forcing(np.zeros(4),np.ones(4)),np.zeros(4))
        np.testing.assert_array_equal(self.network.inertial_forcing([-1.01,0.,0.,0.],np.ones(4)),np.zeros(4))

    def test_asymmetric_damper_switches_halves_and_disabled_flags_are_suppressed(self):
        configured=list(self.struts)
        configured[4]=PhysicalStrut(0.,0.,0.,AsymmetricDamper(),1.,1.,1.)
        configured[5]=PhysicalStrut(0.,0.,0.,AsymmetricDamper(),0.,1.,1.)
        network=ParallelNetwork(self.kin,tuple(configured))
        compression=network.forces(np.zeros(4),[.08,.08,0.,0.])
        rebound=network.forces(np.zeros(4),[-.08,-.08,0.,0.])
        self.assertAlmostEqual(compression.element_force['heave-front'],.16)
        self.assertAlmostEqual(rebound.element_force['heave-front'],-.4)
        self.assertTrue(compression.element_out_of_range['heave-front'])
        self.assertFalse(compression.element_out_of_range['roll-front'])
        disabled_only=network.forces(np.zeros(4),[.08,-.08,0.,0.])
        self.assertFalse(disabled_only.element_out_of_range['roll-front'])
        self.assertFalse(bool(disabled_only.out_of_range[0]))
        self.assertFalse(bool(disabled_only.out_of_range[1]))

    def test_unlimited_travel_and_compile_static_balance(self):
        self.assertTrue(np.all(np.isfinite(self.network.forces([1.1,0.,0.,0.],np.zeros(4)).total)))
        balanced=tuple(PhysicalStrut(1.,p,0.,LinearDamper(0.),0.,1.,1.) for p in (5.,5.,7.,7.,0.,0.,0.,0.))
        self.assertEqual(build_parallel_suspension(kinematics=self.kin,struts=balanced).layout,'parallel-corner-axle')
        with self.assertRaises(ValueError): build_parallel_suspension(kinematics=self.kin,struts=self.struts)

    def test_spring_energy_finite_difference_matches_wheel_force(self):
        z=np.array([.013,-.006,.021,.002]); force=self.network.static_forces(z); epsilon=1.e-7
        for wheel in range(4):
            plus,minus=z.copy(),z.copy(); plus[wheel]+=epsilon; minus[wheel]-=epsilon
            energy=lambda q:sum(.5*s.spring_rate*(self.network.jacobian[i]@q)**2+s.preload*(self.network.jacobian[i]@q) for i,s in enumerate(self.struts))
            self.assertAlmostEqual((energy(plus)-energy(minus))/(2*epsilon),force[wheel],places=7)
