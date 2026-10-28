# Scaled Catalunya GP — uniform 4 m corridor

## Files
- [catalunya_gp_scaled_4m.xml](catalunya_gp_scaled_4m.xml): loadable closed track model, constant half-width 2 m.
- [catalunya_gp_scaled_4m.csv](catalunya_gp_scaled_4m.csv): centreline and analytic heading/curvature in SI units.
- [catalunya_gp_scaled_4m.svg](catalunya_gp_scaled_4m.svg): exact scaled cubic centreline stroked at 4 m width; SVG coordinate units are metres.

## Design and physics
User-selected practical top speed: **99%** of the drag-limited equilibrium speed. Wheel power P = 80000 W; mass m = 270 kg; CdA = 2.0 m². Assumed air density rho = 1.225 kg/m³. Flat road, still air, constant wheel power, no rolling resistance, traction limit, drivetrain loss, gearing limit, or braking distance. This is a sizing exercise, not a drivable lap simulation. Constant power from rest has singular initial force; the energy-domain solution remains finite.

With k = rho CdA / 2 = 1.225 kg/m, drag Fd = k v² and P = F v:

    m v dv/dt = P - k v³
    vmax = (P/k)^(1/3)
    x(v) = -m/(3k) ln(1 - (v/vmax)³)

- Equilibrium speed: **40.2702784689 m/s = 144.9730025 km/h** (unreachable in finite distance).
- Turn 1 entry target: **39.8675756843 m/s = 143.5232725 km/h**.
- Required main-straight centreline arc length: **258.3605801907 m**.
- Ideal acceleration time from rest: **7.7493402168 s**.
- Uniform scale: **0.243907974500 m per source SVG unit**.
- Scaled lap centreline length: **1159.6919136987 m**.
- Uniform track width: **4.0 m**, 2.0 m normal offset on each side, independent of scale.

## Geometry conventions and limitations
Extracted main black cubic path `path3965`, not pit lane, labels, coloured sectors or alternate-layout strokes. Source traversal is reversed to race direction. Start at last-turn exit, source coordinate (2234.2018, 1445.5428); Turn 1 entry is (1174.9743, 1443.9961). Main straight is source cubics 43 and 42 in reverse, with combined source arc length 1059.254338529565 units. These Bézier endpoints define the engineering straight limits; the map has slight drawn curvature even on the straight. No extra turn-exit arc or braking zone is included.

Origin is the last-turn exit. Local `x_north` = negative source SVG y displacement; `y_east` = source SVG x displacement. Axes are map-local, not surveyed geographical north/east. Heading is atan2(east derivative, north derivative), unwrapped; positive curvature turns right. Analytic Bézier derivatives determine heading and curvature; source joins are retained, including any small tangent/curvature discontinuities. Arc lengths use numerical quadrature. Samples are <=0.25 m apart by arc length, include every cubic join and Turn 1 entry, and duplicate the starting coordinate at lap end. Zero smoothing. Width is a design assumption, not an estimate of the real circuit width. The 1,000,000 m radius cap affects legacy radius consumers only, not signed stored curvature.

## Source, attribution and license
Source: [Formula1 Circuit Catalunya.svg](https://commons.wikimedia.org/wiki/File:Formula1_Circuit_Catalunya.svg), by **Will Pittenger**, current source revision **Sentoan, 18 May 2017**. The linked map is the historical GP layout with the final chicane (Commons labels it GP 2007–2020); it is not the current GP layout or a survey.

Original download: https://upload.wikimedia.org/wikipedia/commons/9/9c/Formula1_Circuit_Catalunya.svg

Source SHA-256: `8a0884bdfbccc70a37667ad6e1a08b0a4a1f5929391f791506fb063125d1cbbd`.

The derived centreline, corridor and rendering are licensed **[CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/)**. Changes: extraction, direction reversal, translation, uniform scaling, analytic resampling, and replacement of the source drawing width with a uniform 4 m corridor. Built solely with one-off commands; no persistent generator script added.
