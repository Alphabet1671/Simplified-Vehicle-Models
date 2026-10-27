# Stage 3 interval-oracle local-coordinate repair

## Failure diagnosis

The MIS 0.5 m suite failed at segment 153 with
`interval_oracle_unresolved`, cell `(18, 25, 286)`, and a strict-versus-
polynomial moment discrepancy of `1.113e-4` at speed `12.8724009129`.

The prior oracle expanded a multilinear cell margin in global speed `u` and
then composed it with `u = a + (b-a)r` for root isolation. In narrow,
dense-map cells this high-degree global power basis suffers cancellation. The
post-root guard evaluated that unstable global expansion while strict map
evidence remained the authoritative evaluator.

## Repair

Cell moments and demand are now assembled directly in local `r in [0,1]`, with
`u = a + (b-a)r`. Root isolation consumes that local polynomial directly, and
the unchanged strict-evidence guard evaluates the local polynomial at the
normalized witness. No yaw-moment, root, support, interpolation, or map
acceptance tolerance was changed. Disagreement still fails closed.

## Verification

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_yaw_profile_transition -q
```

Result: 17 tests passed in 1.078 seconds; `git diff --check` passed. The suite now includes a narrow high-speed speed/acceleration/lateral-cell regression with large multilinear moment variation; it verifies returned interval endpoints and midpoint witnesses through the strict edge oracle.

The full guarded MIS 0.5 m suite reproduction remains a parent-owned physical
run, to write under a fresh `mis-fixed-0p5m-local` output path.
