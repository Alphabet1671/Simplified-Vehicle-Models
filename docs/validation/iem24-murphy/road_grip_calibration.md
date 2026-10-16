# IEM24 Murphy proposed road-grip baseline

## Status

**Proposed baseline. Not a measured or validated road coefficient.** No
measurement in this repository establishes the road friction of the IEM24
tire. The values below are a scaling of the lab fit by a published team prior,
checked for plausibility against one measured lap set. Treat results that use
them as a sensitivity study around a stated prior.

## Decision

`models/tires/iem24_murphy_real_unitire.xml` uses one shared factor of **2/3**
on the four friction parameters of the lab fit
`models/tires/hoosier-43075-r20-simple.xml` (lines 26-33). Slip stiffness,
shape, trail, and every other parameter are unchanged.

| Parameter | Lab fit | Proposed road (2/3) | Previous provisional |
|---|---|---|---|
| `mu_0_x` | 2.3361624600071096 | 1.5574416400047397 (~1.55744) | 0.80 |
| `mu_s_x` | 2.0159412226800204 | 1.3439608151200135 (~1.34396) | 0.80 |
| `mu_0_y` | 2.3803804777987394 | 1.586920318532493 (~1.58692) | 1.50 |
| `mu_s_y` | 2.053563821658717 | 1.3690425477724781 (~1.36904) | 1.50 |

The change raises the previous provisional longitudinal grip from 0.80 to
about 1.557 (peak) and lateral from 1.50 to about 1.587.

## Source of the 2/3 prior

Milliken Research, *TTC Round 9* contents,
<https://www.millikenresearch.com/Contents_Round9.pdf>. The team-reported
guidance is that the TTC belt surface is smoother than a real track, so road
grip is about two thirds of the lab value. The factor is used as that prior
only. It is **not** inferred from the 120-grit belt label, no universal
grit-to-asphalt constant is claimed, and the vehicle data do not validate it.

## Measured check (what the lap does and does not say)

Effective grip is `m|a| / (m g + 0.5 rho CL A v^2)` with `m = 287 kg`,
`rho = 1.22 kg/m^3`, `CL A = 3.5 m^2` (`downforce_area`) from
`docs/validation/iem24-murphy/measured_stint.csv` (16167 samples).

- Lateral: the peak effective grip is about **1.43**. In this CSV the 99.5th to
  99.9th percentiles are 1.39 to 1.46; the single-sample maximum is 1.69 and
  is treated as an outlier. Against the lab lateral peak 2.38 this implies a
  lab-to-road ratio of about **0.60**.
- Braking: the peak effective longitudinal deceleration is **below 1** (about
  0.64 to 0.67). The driver does not brake at the tire limit, so this is a
  lower bound on longitudinal grip and does not identify the ratio.

Consequences:

- The proposed lateral peak 1.587 is **optimistic** relative to the measured
  ~1.43 (a 2/3 ratio against an estimated ~0.60). The measured value is itself
  a lower bound of available grip, because a driver need not sit at the limit.
- The **longitudinal ratio is unidentifiable** on the current lap, since the
  braking peak is below 1 and traction is power limited. 2/3 is applied to x by
  assumption.

## Major proxy disclosure

The longitudinal lab data, TTC `B2356` run 72, belong to a **43100 R20** tire,
not the 43075 tire of run 8 (lateral). The x coefficients of the lab fit, and
therefore the x coefficients here, are a cross-tire proxy. See the identity
correction in `docs/validation/unitire-simple.md`.

## What is not claimed

- No measured road coefficient is established.
- The 2/3 prior is not validated by the vehicle and not derived from
  120-grit belt data.
- No lateral/longitudinal split beyond the shared factor is supported.
- The legacy fitted lab XML is unchanged and stays the lab reference.

## Verification

```bash
PYTHONPATH=src python3 -m unittest tests.test_iem24_road_grip tests.test_unitire_simple -v
```

`tests/test_iem24_road_grip.py` checks the exact 2/3 ratio against the lab
XML, that all non-friction parameters are unchanged, the friction cap
(peak force per load never above the new `mu_0`, combined force within the
friction ellipse), and that the lab XML still holds the original fit.
