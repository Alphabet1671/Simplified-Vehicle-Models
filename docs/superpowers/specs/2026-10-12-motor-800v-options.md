# Architecture-compatible 800 V motor-model options

Status: proposal only. The current production cap stays at 600 V. The LUT has been promoted independently of this proposal.

## Finding: solve a voltage inequality, not an equality

`solve_direct_current` currently solves the quadratic boundary

```text
a id² + b id + c = 0
```

and selects the smallest-magnitude root even when no field weakening is needed. The machine constraint is instead

```text
vd = R id - ωe Lq iq
vq = R iq + ωe (ψ + Ld id)
vd² + vq² <= Vavailable²
```

Thus `c` is the voltage residual at `id = 0`. If `c <= 0`, zero d-axis current is already feasible. Forcing a boundary root invents a large current, increasing with voltage headroom; changing root selection alone does not solve the problem. When zero is infeasible, choose the minimum-magnitude point in the feasible quadratic interval. This is a minimum-|id| rule at the existing fixed-iq torque approximation, not a full MTPA controller.

A read-only experiment installed `if Vavailable < 0: infeasible; elif c <= 0: id=0; else: use the existing boundary roots` in memory. No production code was changed.

| Bus voltage | Speed (rpm) | Existing zero-torque bus draw (W/motor) | Candidate draw (W/motor) |
|---|---:|---:|---:|
| 650 V | 0 | 129,629.39 | 0.00 |
| 650 V | 145 | 124,919.29 | 3.57 |
| 650 V | 1,000 | 45,890.66 | 23.17 |
| 650 V | 18,000 | 1,651.69 | 1,565.57 |
| 800 V | 0 | 580,739.67 | 0.00 |
| 800 V | 145 | 559,682.73 | 3.57 |
| 800 V | 1,000 | 206,135.27 | 23.17 |
| 800 V | 18,000 | 2,019.77 | 1,565.57 |

At 600 V zero-torque bus draw is unchanged, but the spurious reported d-axis current becomes zero. This does not prove that all loaded 600 V points remain unchanged.

A 240-point voltage/speed/signed-torque diagnostic (400–800 V, 0–18,600 rpm, −20 to +20 Nm) found no voltage-inequality violations among candidate-feasible points. Two points were feasible at the actual higher voltage but infeasible at the 600 V reference: (700 V, 15,000 rpm, −20 Nm) and (800 V, 15,000 rpm, −20 Nm). These are solver probes, not assertions that those points satisfy every assembled rating. They expose another blocker: `evaluate_shaft_power_exact` currently rejects an operating point if its reference-voltage solve is infeasible, even when the actual-voltage solve is feasible.

The earlier 129,629 W versus 580,740 W reports were different voltages: 650 V versus 800 V, not two answers for the same point.

## Option 1 — inequality solve + conservative reference-map losses (recommended)

Preserve the existing dataclasses and all forward, limits and inverse signatures. Change the shared analytical source, then rebuild the LUT from that source.

1. In `solve_direct_current`, reject negative available voltage; test whether `id=0` is feasible before calculating boundary roots. For a binding voltage constraint, use the minimum-magnitude feasible interval point. Use stable quadratic roots and a scale-aware discriminant tolerance, and verify the returned point satisfies the voltage inequality. Preserve signed-iq drive/regen handling and standstill behaviour.
2. In `evaluate_shaft_power_exact`, actual-voltage infeasibility remains authoritative. Reference infeasibility is not a reason to reject a higher-voltage feasible point.
3. At or above `reference_bus_voltage`, use the source loss map without an added field-weakening penalty. Below reference, retain the conservative added-copper-loss form, but use the corrected actual/reference currents:

```text
Vactual = min(Vterminal, maximum_bus_voltage)
if actual solve is infeasible: return infeasible
if Vactual >= Vreference:
    loss = map_loss
else:
    reference solve must be feasible
    loss = map_loss + max(1.5 R (id_actual² - id_reference²), 0)
```

4. Compute the calibration/reference solve at the fixed identification voltage, not at the regulatory-clamped voltage. Introduce an internal voltage conversion helper for that calibration calculation; actual-bus paths must still use `limit_bus_voltage`. This also avoids silently moving the reference when the configurable regulatory limit is below 600 V.
5. Keep the source torque-speed envelope, absolute speed limit and per-unit power limit. Confirm the existing voltage-scaled cap and electrical feasibility both apply in drive and regen. Do not infer higher torque, motor speed, or insulation capability merely from a larger regulatory voltage limit.

**Why it fits:** the generator, exact fallback and analytical mode already share `evaluate_shaft_power_exact`; runtime LUT lookup and direct inversion do not need a new state variable or solver. It removes the fake high-voltage loss and reference-feasibility veto without pretending we know how much measured loss improves above the reference.

**Tradeoff:** conservative energy prediction above reference; it gives no efficiency credit for reduced real field-weakening current. Loaded 600 V and below-reference results may change because the d-axis solution becomes physically meaningful. Revalidate and rebaseline rather than preserving incorrect results.

## Option 2 — inequality solve + calibrated signed copper-loss correction

Use the same inequality solver, but allow reference-relative copper loss to decrease with voltage headroom:

```text
base_loss = map_loss - estimated_reference_d_axis_copper_loss
loss = base_loss + estimated_actual_d_axis_copper_loss
```

This needs a defensible decomposition of measured reference losses and a nonnegative residual model. Do not simply subtract a guessed copper term and clip total losses to zero: that can conceal inconsistent identification or missing iron/inverter losses. Where the reference solve is infeasible, require a calibrated baseline/decomposition or explicitly mark the source-data domain unsupported rather than inventing a reference current.

**Why it fits:** still produces the same bus-power/loss arrays and works with the existing forward/inverse APIs and exact fallback.

**Tradeoff:** more realistic 800 V efficiency only after new calibration data or validated loss decomposition. The current 600 V data alone do not establish accurate 800 V energy use. Requires source/provenance additions, possibly new compiled loss parameters.

## Option 3 — current-constrained dq operating-point optimization

Replace fixed `iq = T/(1.5 p ψ)` with a minimum-loss dq operating-point solve including reluctance torque, current magnitude limits, voltage limits, and flux/field-weakening control. Add explicit machine/inverter current and hardware-voltage ratings, and any calibrated loss coefficients, to XML compilation.

The optimizer runs offline while generating the LUT and as the out-of-domain exact fallback. Runtime forward and direct inverse queries retain their interfaces.

**Tradeoff:** strongest physics, but new identification data and considerably more implementation/validation; exact fallback can become slower. Appropriate for a genuinely new 800 V drivetrain, not necessary to correct this source-model bug now.

## Integration and acceptance for any option

- Keep `maximum_bus_voltage` (competition/model limit), `reference_bus_voltage` (600 V identification baseline), actual terminal voltage, and hardware rating distinct. An 800 V rule does not rate the present AMK motor, inverter, or battery for 800 V.
- Keep physical battery voltages/current/SOC untouched. A software motor-voltage clamp does **not** make a 608.7 V pack electrically compliant with a 600 V regulation or represent a real DC/DC converter; flag regulatory violations separately. The current pack data must be redesigned or restricted if compliance is required.
- First land the analytical-source fix with the limit still 600 V. Replace the existing known-defect guard with small-draw, headroom, signed-torque and feasibility regression tests. New high-voltage tests should use an explicitly configured 800 V test model.
- Rebuild the 600 V artifact as well as a separate 800 V configuration/artifact; never reuse the existing LUT after changing source equations. Bump the generator version and record solver version/configuration/source digests. Validate configuration association at load time, not just byte integrity, so an old but correctly hashed table cannot silently coexist with a corrected exact fallback.
- Choose the 800 V axis from the target pack's measured loaded-voltage range, with margin and maximum at its configured limit; do not automatically stretch the existing 530–600 V grid. Retain exact out-of-domain fallback.
- Probe all drive rows for monotonicity and preserve complete non-monotone regen curves. If a corrected drive row is genuinely non-monotone, do not collapse its torque envelope to approximately zero to enforce lookup assumptions: generalize drive inversion to scan segments or route explicitly to exact evaluation, with observable counters.
- Test voltage headroom (`id=0` when feasible), continuity at the onset of field weakening, boundary feasibility, negative available voltage, signed torque, zero speed, above-reference actual feasibility, reference calibration with limits below/above 600 V, and finite nonnegative loss with consistent directional inverter power.
- Require deterministic regeneration, analytical/LUT operating-point agreement, zero pack-power violations, cap/corner coverage, and matched 800 V lap/energy comparisons. Reapply lap delta ≤1 ms, maximum speed displacement ≤0.05 m/s, both converged, and a measured runtime win. Validate recovered energy in joules separately: the previously identified SOC/capacity accounting issue is not fixed by this motor change.

**Recommendation:** implement Option 1 first, retain the 600 V production limit while validating it, then enable an explicitly configured 800 V model/artifact only after hardware and source-data checks. Choose Option 2 when calibrated efficiency data become available; reserve Option 3 for a higher-fidelity drivetrain model.
