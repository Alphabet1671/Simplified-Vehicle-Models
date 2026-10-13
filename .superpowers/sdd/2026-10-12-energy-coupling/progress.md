# Energy-coupling redesign ledger

Branch: `feature/energy-coupling` (from pushed `feature/powertrain` @ 1317110).
Scope: `docs/superpowers/specs/2026-10-12-energy-coupling-scope.md`.
Baseline: `baseline-22lap.md` in this directory (11 laps, 2,972.7 s cumulative).
Subagent model: Ling 3.0 Flash Fin via OpenRouter (user instruction).

## Sequence

W0 (diagnostics) → Gate A → W1 (adaptive coupling) → Gate B → W2/W4/W5 as
directed → W3 (joint fixed point) if still required → W6 (validation).

## Gate A decision rules — PRE-REGISTERED before W0 data arrives

Registered 2026-10-12, before the W0 report was read, so the gate decision is
mechanical rather than post-hoc.

**D1 — iteration mechanism.**
Confirmed if the per-iteration residual sequence r_k decays with
max(r_{k+1}/r_k) in [0.35, 0.65] across the three SOC runs — i.e. the 0.5
relaxation of a contractive map sets the count, not the physics.
- Confirmed → W1 (adaptive/predicted update) attacks iteration count directly.
- Residual stalls or oscillates → the map is not contractive as assumed; W1
  must be damping-adaptive by necessity and W3 gains priority.

**D2 — wall-time split.** Let f_solve = fraction of per-lap wall inside
solve_lap calls, f_energy = fraction inside _energy_trace, f_other = remainder.
- f_solve > 0.6 → iteration count is the primary target; W1 first.
- f_energy > 0.3 → W4 (energy path on the LUT) is co-primary; both W1 and W4
  are required for the 5x target.
- These are not mutually exclusive; both may bind.

**D3 — SOC dependence / LUT floor.**
Trigger W4's voltage-axis extension as MANDATORY if the SOC 0.25 run shows any
of: LUT fallback counts > 10x the SOC 0.96 run; minimum bus voltage < 530 V;
or wall time > 1.5x the SOC 0.96 run.
Prior evidence already favours triggering: the baseline's later laps cost 2.3x
at identical iteration counts, which iteration count cannot explain.

**Gate A output — O1-only vs O1+O2.**
- Project W1 gain as (current 15 passes/lap) / (projected 4-6 passes).
- Project W4 gain from the oracle-call and fallback reductions it removes.
- If projected(W1) x projected(W4) >= 5x → defer or skip W3 initially and
  re-decide at Gate B with measured numbers.
- If W1 alone projects < 3x AND f_energy < 0.3 → W3 is required.

Any deviation from these rules once the data arrives must be recorded here
with the reason.

## History

- 1317110 scope doc committed; `feature/powertrain` pushed to origin (user
  instruction: push the working-but-slow model first).
- a0e5c0f baseline recorded. 22-lap run truncated at 11 laps on user
  instruction. Per-lap details beyond wall time and iteration counts were lost
  (script buffered them until completion); lesson recorded — diagnostics must
  flush incrementally.
- W0 dispatched to Ling 3.0 Flash Fin (workflow-63). Machine left uncontended
  for its timing runs.
