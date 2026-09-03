# Airspace Exposure-Rate Model (Layers 1+2) — Working Spec

**Status:** Living document, first draft. Companion to `uas_midair_collision_model_spec.md` (the fine-grained, per-encounter collision-probability model). This document covers the *precursor* question that model explicitly set aside: **how often do two drones end up in the overlapping-volume situation at all?**

**Relationship between the two models:** the fine-grained model answers P(collision | overlap, K independent encounters), with K currently an exogenous slider. This model's output — an annualized rate of qualifying encounters — is the natural candidate to eventually *compute* K from real airspace/operational assumptions rather than leaving it as a free input. That integration is not yet built; this document describes the rate model on its own.

---

## 1. Scope and the Three-Scenario Framing

The airspace is governed by UTM strategic deconfliction. Three scenarios define what is and isn't in scope:

1. **Scenario 1** — A delivery route in Accepted state (pre-takeoff) that would be preempted by a DFR route already in Accepted state: **out of scope**. The delivery operator is required to plan a new, non-conflicting route before ever launching.
2. **Scenario 2** — A delivery route and a DFR route, both in Accepted state, that simply don't intersect: **out of scope**, no assumed collision risk. This is expected to be the large majority of flight pairs.
3. **Scenario 3 (the case this model addresses)** — A delivery drone is already **Activated** (airborne, in progress) when a DFR drone launches, and the DFR's operation subsequently intersects the delivery drone's *remaining* route. This can only happen to already-flying drones — a delivery route still in Accepted state at the moment of DFR launch would itself be caught by Scenario 1's strategic deconfliction check and never launch into conflict. This model computes the **rate** at which Scenario 3 situations occur, not the probability of collision given one occurs (that's the fine-grained model's job). Scenario 3 is evaluated as three sub-cases:
   - **Scenario 3a** — overlap during a delivery **cruise** segment with a DFR's **loiter** footprint. Vertical separation is a genuine protective factor here. See Section 7 for the geometric test and Section 5 for the vertical gate.
   - **Scenario 3b** — overlap during a delivery **hover** (pickup/dropoff) with a DFR's **loiter** footprint. No effective vertical gate, since the drone/payload sweeps the full altitude range while descending. See Section 7 for the geometric test and Section 5 for the vertical-gating rationale.
   - **Scenario 3c** — overlap between the DFR's **straight-line transit** phase (dock → call site) and a delivery route (cruise or hover). Uses a fixed, much smaller combined corridor buffer than Scenario 3a/Scenario 3b's loiter footprint, and is reported as its own standalone rate, not carried into the fine-grained model. See Section 7.1 for full detail.

Explicitly **out of scope** for this model:
- Ground risk.
- The actual fine-grained collision probability once a Scenario 3 situation is identified (handled by the companion model).

---

## 2. Region and Actors

- **Operational region:** a disk of radius 5 miles, centered on a single drone delivery hub.
- **Delivery operator:** one hub, launching a variable number of deliveries per day (100–5,000, slider). Cruise speed fixed at 45mph. Hover duration at each stop fixed at 45 seconds.
- **DFR (public-safety) operator:** 1–6 fixed docks within the same region, each independently responding to 1–20 calls/day (slider). DFR transit speed fixed at 45mph (out of scope for collision, in scope for timing). Loiter duration at the call site swept 5–30 minutes.

### 2.1 Dock placement

Docks are placed **deterministically**, not randomly: N docks at a fixed radius of 60% of the region radius (3 miles) from the hub, spaced at equal angular intervals (360°/N apart). This achieves "evenly spaced, not clustered, not on the perimeter" by construction rather than by a random process that could still (rarely) violate that intent. For N=1, the single dock is placed at a fixed reference angle at the same radial distance.

---

## 3. Delivery Route Models

Two route topologies are modeled, since the choice was shown to matter through two separate mechanisms (not one): flight duration (→ concurrency, via Little's Law) and spatial coverage pattern (single-hub spokes vs. free-floating chords).

### 3.1 Out-and-back
Hub → random delivery point (uniform in disk) → hover 45s → hub.

### 3.2 Triangular
Hub → random pickup point (uniform in disk) → hover 45s → random dropoff point (uniform in disk, independent of pickup) → hover 45s → hub.

### 3.3 Expected trip duration (closed-form, used as a validation cross-check)

Using the standard results for a uniform random point in a disk of radius R — E[distance from center] = (2/3)R, E[distance between two independent uniform points] = (128/45π)R ≈ 0.9054R — at R = 5 miles:

| Route type | E[total distance] | Cruise time @ 45mph | Hover time | **E[trip duration]** |
|---|---|---|---|---|
| Out-and-back | 6.67 mi | 8.89 min | 45s (1 stop) | **9.64 min** |
| Triangular | 11.19 mi | 14.92 min | 90s (2 stops) | **16.42 min** |

Simulated route durations matched these closed-form values almost exactly during validation (9.66 min and 16.49 min observed vs. 9.64 and 16.42 predicted), confirming the route-sampling code is correct.

**Implication:** triangular routing produces ~1.7x longer average trip duration for the same daily delivery count, which — via Little's Law (L ≈ λW) — directly means more concurrently-airborne delivery drones at any instant, and therefore more Scenario 3 exposure, independent of any other parameter change.

### 3.4 Cruise altitude

Each delivery route independently draws a cruise altitude uniformly from a **discrete set: {200, 250, 300, 350, 400} ft**. This was added specifically so that some delivery flights cruise entirely above or below where a given DFR is loitering, producing genuine full vertical separation from geometry alone — not just from the vertical-margin check (Section 5).

---

## 4. Delivery and DFR Arrival Timing

Both delivery deliveries and DFR calls arrive over a fixed **8am–8pm operating window** (12 hours), per an explicit worst-case simplifying assumption that all operations are confined to this window.

### 4.1 Delivery timing — two selectable models (toggle)
- **Peaked (default):** 60% of daily deliveries occur within two 2-hour peak windows (11am–1pm, 5pm–7pm; 4 of the 12 operating hours combined), split between the two windows proportional to their length. The remaining 40% are spread uniformly across the other 8 hours. The 60% figure is an adjustable placeholder, not derived from real operational data.
- **Uniform:** all deliveries spread uniformly across the full 12-hour window, no peak structure.

### 4.2 DFR call timing — currently uniform only, architected for extension
DFR calls are currently modeled as arriving **uniformly** across the operating window (no peaked structure), since 911 call volume presumably doesn't track delivery demand patterns. The timing-generation function is shared code between deliveries and DFR calls (`delivery_arrival_times()`, shared between both), so adding a different DFR-specific temporal distribution later does not require rebuilding the tool — only adding a new branch to that one function.

### 4.3 911 call locations
Uniform over the **entire** region disk, regardless of which dock ultimately responds — not restricted to a sub-region nearer any particular dock. This is a stated simplifying assumption based on the information available; if call-to-dock assignment should be distance-weighted or otherwise non-uniform, this is the place to revisit.

---

## 5. Vertical Model

This is a **coarser** vertical model than the fine-grained model's, deliberately — this layer's job is to determine whether a Scenario 3 situation is even geometrically plausible, not to resolve fine collision probability (that's downstream).

- **DFR loiter altitude:** drawn **uniformly** within the declared band (100–300ft) independently per call. Critically, this is **not** worst-case-centered on the delivery cruise altitude — this was a specific, flagged correction, since worst-case-centering here would silently make vertical separation impossible by construction, contradicting the requirement that some Scenario 3 candidate pairs have genuine z-offset and no real conflict.
- **Cruise-phase (Scenario 3a) vertical gate:** requires `|DFR loiter altitude − this flight's cruise altitude| ≤ vertical_margin_ft`. Default `vertical_margin_ft = 50`, an **adjustable placeholder assumption**, not a validated threshold. With cruise altitude now varying over {200,...,400}ft, the average pass probability for this gate (holding DFR altitude uniform over 100-300ft) is approximately 25% across the five cruise-altitude options (200ft and 250ft can fully satisfy it; 300ft partially; 350/400ft essentially never) — confirmed by direct simulation (see Section 8).
- **Hover-phase (Scenario 3b) vertical gate:** **none.** During pickup/dropoff, the delivery drone (or its payload) descends from cruise altitude toward the ground and back, sweeping through the DFR's entire possible altitude band — so vertical separation provides no protection during this phase. This was an explicit correction from the user, since it's a materially different risk mechanism than cruise-phase exposure and a significant source of exposure time if a hover point happens to fall within a DFR's loiter footprint.

---

## 6. DFR Loiter Footprint Model

The DFR's loiter footprint is modeled as a **static circular disk** at the 911 call location — not the actively orbiting motion used in the fine-grained model. This is a deliberate simplification specific to this layer: Layer 2's job is to produce a coarse space-time overlap **rate**, which then feeds the fine-grained model (which owns the actual orbiting-collision geometry) — modeling orbiting motion redundantly here would add complexity without changing what this layer needs to answer.

**Footprint radius is swept across four values: 50m, 150m, 300m, 500m**, matching the same reference points used for the orbit-radius-extent slider in the fine-grained interactive tool, for consistency between the two models.

---

## 7. Geometric/Temporal Intersection Test

For each (delivery flight, DFR event) pair, per footprint radius:

1. **Temporal pre-filter (cheap):** does the delivery flight's total active window (from its Activated launch time to its return) overlap the DFR's loiter window at all? If not, skip — this eliminates the large majority of pairs cheaply before any geometry is evaluated.
2. **Per-segment exact geometric solve:**
   - **Cruise segments** (the drone moving in a straight line between two points at constant speed): solved analytically as a closest-point-on-a-line-*segment*-to-a-fixed-point problem (a bounded quadratic solve, not an infinite line), yielding the exact time sub-interval — if any — during which the drone is within the footprint radius. This sub-interval is then intersected with the loiter window; a non-empty intersection, combined with a passing vertical gate (Section 5), registers a **Scenario 3a** event.
   - **Hover segments** (the drone stationary at a point): distance to the DFR location is constant, so the check is a single fixed-point-in-circle test; if true, the entire hover interval is checked against the loiter window for temporal overlap. A non-empty intersection registers a **Scenario 3b** event (no vertical gate).
3. **Scenario 3a and Scenario 3b are tracked and reported separately** (not summed into one number by default), specifically so their different risk mechanisms and relative contributions can be examined independently before any decision to combine them for a headline metric.

No time-discretization is used anywhere in this test — all segment-level geometry is solved exactly,.

### 7.1 Sub-case Scenario 3c: DFR Transit Phase vs. Delivery Route

The DFR's straight-line transit leg (dock → call location, 45mph) is checked against delivery routes as its own sub-case, separate from Scenario 3a/Scenario 3b and **not** carried forward into the fine-grained model's K parameter — reported as a standalone rate.

**Geometry:** unlike Scenario 3a/Scenario 3b (a moving delivery drone vs. a *static* DFR footprint disk), Scenario 3c is the first case in this model where **both** sides can be moving simultaneously (delivery cruising, DFR transiting) or one moving against one stationary (delivery hovering, DFR transiting). Since both legs move at constant velocity within their respective segments, relative position is affine in time within any temporal overlap window, making squared separation a quadratic in time — solved in closed form exactly (`two_segment_intersection_window()`), with no time-discretization. This single function correctly degenerates to the point-vs-moving-point case when one side is stationary (hover), so one implementation covers both cruise-vs-transit and hover-vs-transit.

**Threshold:** a **fixed combined TSE-style buffer**, not part of the loiter-footprint radius sweep — 5m (delivery) + 5m (DFR transit) = **10m combined horizontal**, 10m combined vertical (≈32.8ft), reusing the fine-grained model's half-width convention for both aircraft rather than the loiter footprint's much larger 50-500m radii (loitering and transiting are different behaviors with different appropriate corridor sizes).

**Vertical gating:** DFR transits at the **same altitude as its corresponding loiter** (already drawn per-call). Cruise-vs-transit applies the combined vertical buffer as a gate; hover-vs-transit has **no** vertical gate, for the same reason as Scenario 3b (the delivery drone/payload sweeps the full altitude range during descent).

**Validation performed:** near-zero and very-large combined-buffer edge cases behave as expected (≈0 and many events respectively); and a hand-computable unit test — two points closing head-on at a combined 20 m/s with a 1m threshold — produced a threshold-crossing window of exactly 0.1s (= 2×threshold/closing-speed) centered exactly at the analytically expected meeting time, confirming the closed-form solver is exact.

**Reference-scenario result** (same parameters as Section 9): **0.107 ± 0.040 events/day (95% CI), ≈39/year** — roughly 6x smaller than Scenario 3a at the tightest loiter radius (50m) and about 90x smaller than the combined Scenario 3a+Scenario 3b total at the widest loiter radius (500m). Small but clearly nonzero, matching the expectation that motivated adding this sub-case.

---

## 8. Validation Performed

- **Dock placement:** confirmed exactly equidistant from hub and correctly angularly spaced for N=3.
- **Near-zero footprint radius:** produces ~0 events (measure-zero geometric event), as expected.
- **Very large footprint radius (larger than the region):** produces a large, non-trivial event count, gated only by temporal overlap and the vertical check — as expected.
- **Zero vertical margin:** correctly suppresses *all* Scenario 3a events to exactly 0, while leaving Scenario 3b events unaffected — confirms the two gating logics are implemented independently and correctly.
- **Nonzero vertical margin with varying cruise altitude:** produces a reduced Scenario 3a count consistent with the hand-computed ~25% average vertical-gate pass probability across the five discrete cruise altitudes (200/250ft largely pass, 300ft partially, 350/400ft essentially never) — matches expectation.
- **Route duration cross-check:** simulated mean trip durations for both route types matched the closed-form Little's-Law-input calculation (Section 3.3) to within simulation noise.
- **Runtime:** a full simulated day at maximum slider values (5,000 deliveries, 6 docks × 20 calls/day = 120 DFR events) completes in ~0.4 seconds, confirming that a genuine multi-hundred-day Monte Carlo run (needed for confidence intervals, not a single deterministic day) is computationally cheap.

No independent-of-model cross-check has been performed beyond the rough order-of-magnitude back-of-envelope check described in Section 9 — this should be treated as directionally reassuring, not as rigorous validation.

---

## 9. Reference Scenario Results

**Parameters:** 1,000 deliveries/day, out-and-back routing, peaked delivery timing, 3 docks × 8 calls/day (uniform DFR timing), 15-minute loiter, vertical margin ±50ft, 300 simulated days.

| Footprint radius | Scenario 3a/day (mean, 95% CI) | Scenario 3b/day (mean, 95% CI) | Scenario 3a+Scenario 3b annualized (×365) |
|---|---|---|---|
| 50m | 0.65 ± 0.12 | 0.03 ± 0.02 | ~249 |
| 150m | 2.13 ± 0.29 | 0.21 ± 0.05 | ~852 |
| 300m | 4.58 ± 0.61 | 0.71 ± 0.10 | ~1,933 |
| 500m | 7.70 ± 0.73 | 1.91 ± 0.16 | ~3,510 |

**Scenario 3c (DFR transit vs. delivery route, fixed 10m combined buffer, not part of the radius sweep):** 0.107 ± 0.040 events/day, ≈39/year (32 total events observed across 300 simulated days).

**A rough independent plausibility check** (concurrency via Little's Law × approximate path-corridor area coverage × vertical-gate probability) on the 300m/Scenario 3a figure gave ≈5/day against a simulated 4.58/day — good order-of-magnitude agreement, treated as reassuring rather than as rigorous validation, since the back-of-envelope method ignores size-biased "caught mid-flight" sampling and real path geometry.

**Notable finding:** Scenario 3b's share of the total grows as footprint radius increases (from ~4% of the 50m total to ~20% of the 500m total) — worth watching as the full parameter sweep is built out, since it means the hover/descent risk mechanism is not uniformly small across the parameter space, despite hover being a small fraction (~8-9%) of total trip time.

---

## 10. Annualization Assumption

All annualized figures assume **365 operating days per year**, with each day using the same fixed 8am–8pm operating window (Section 4). This is a stated worst-case simplification, not a claim about actual planned operating calendars — flagged here and intended to also appear as a visible footnote in any future interface built on this model, per explicit instruction.

---

## 11. Parameter Reference

| Parameter | Range / Options | Status |
|---|---|---|
| Region radius | 5 miles | Fixed |
| Delivery cruise speed | 45 mph | Fixed |
| Delivery hover duration | 45 sec | Fixed |
| Delivery route type | out-and-back / triangular | Toggle |
| Deliveries/day | 100–5,000 | Slider |
| Delivery timing | peaked (60% in 2 windows) / uniform | Toggle |
| Delivery cruise altitude | {200,250,300,350,400} ft, drawn per-flight | Fixed discrete set |
| Number of DFR docks | 1–6 | Slider |
| Dock placement | Fixed radius (60% of region), even angular spacing | Fixed method |
| Calls per dock per day | 1–20 | Slider |
| DFR call timing | uniform (extensible) | Fixed for now, architected for extension |
| DFR call location | uniform over region | Fixed method |
| DFR transit speed | 45 mph | Fixed, out of scope for collision |
| DFR loiter duration | 5–30 min | Slider (sweep) |
| DFR loiter altitude | uniform, 100–300 ft, drawn per call | Fixed method |
| Vertical margin (Scenario 3a gate) | ±50 ft | Adjustable placeholder |
| DFR footprint radius | 50 / 150 / 300 / 500 m | Sweep (matches fine-grained model's radius reference points) |
| Scenario 3c combined buffer (transit vs. delivery) | 10m horizontal / 10m vertical (fixed) | Fixed, not part of the radius sweep |
| Operating window | 8am–8pm, 365 days/yr | Fixed worst-case assumption |

---

## 12. Sweep Analysis Findings

All results below use out-and-back routing, 300m footprint radius, and 15-minute loiter as the reference point unless a parameter is the one being varied. Scripts: `sweep_runner.py`, `phase1a.py`, `phase1b.py`, `phase2.py`; raw results in the corresponding `phase*_results.json`.

### 12.1 Deliveries/day — linear (R² = 0.999)

Swept 100–5,000 deliveries/day (6 points, 200 simulated days each). Rate scales linearly with deliveries/day for all three sub-cases (Scenario 3a R²=0.9986, Scenario 3b R²=0.9983, Scenario 3c R²=0.9980, all via through-origin regression). **No dense grid is needed on this axis** — a single per-delivery rate multiplier, computed once, extrapolates reliably across the full slider range.

### 12.2 DFR dock count × calls/dock — only the TOTAL matters

Tested four different (docks, calls/dock) splits all producing the same total of 24 events/day: (1,24), (2,12), (3,8), (6,4). Resulting Scenario 3a rates (4.35, 4.78, 4.26, 4.29) all overlapped within confidence intervals — **the split does not matter independently, only the product (total DFR events/day) does.** This is an emergent property of the current model (call locations are uniform over the whole region regardless of which dock responds, so by symmetry no dock-count-dependent effect survives).

Separately, sweeping the *total* (1, 20, 24, 120 events/day) showed the same strong linearity as deliveries/day (R²=0.9995).

**Combined implication:** the two linear findings together support a simple multiplicative model, `rate ≈ C × deliveries_per_day × total_DFR_events_per_day`, for a fixed footprint radius/route-type/timing/loiter combination. Cross-checking the two independently-fit slopes (0.1830/1000 = 0.000183 implied constant from the DFR-events fit, vs. 0.004507/24 = 0.000188 implied from the deliveries fit) agreed within ~2.6% — good support for treating this as a genuine multiplicative structure rather than two separate linear effects that happen to coexist.

**This is a significant simplification for any future interactive tool:** the (deliveries/day) × (docks) × (calls/dock) three-parameter space collapses to a single derived quantity (deliveries/day × total DFR events/day) with one fitted constant per (route type, timing, loiter, radius) combination — no dense 3D grid required.

### 12.3 Route type — ratio matches the Little's Law prediction

Triangular routing produced a Scenario 3a rate 1.625x higher than out-and-back at matched deliveries/day, closely matching the 1.703x ratio predicted purely from the two route types' closed-form expected trip durations (Section 3.3). This confirms route type's entire effect on this model runs through concurrency (via Little's Law) — there is no additional, separate spatial-coverage effect large enough to show up beyond that duration-driven multiplier, at least at this reference scale.

### 12.4 Delivery timing shape (peaked vs. uniform) — no effect, with a clean explanation and a confirmed exception

Peaked (60%-in-two-windows) and uniform delivery timing produced statistically indistinguishable Scenario 3a rates (3.923 vs. 3.927, well within noise), despite peaked timing producing much higher *instantaneous* concurrency during the peak windows.

**Why:** total expected exposure over a full day is proportional to the *integral* of concurrency over the day, and by Little's Law, concurrency(t) ≈ arrival-rate(t) × trip-duration. Integrating arrival-rate(t) over the full day recovers the *total* daily delivery count regardless of how that arrival rate is distributed across the day (peaked or uniform) — so as long as DFR call arrivals stay uniform (unweighted by time), the exposure integral is invariant to the delivery timing shape.

**This invariance is conditional, not universal — confirmed directly:** when DFR call timing was also set to the same peaked shape as deliveries (testing the "future alternate DFR timing distribution" extensibility hook built earlier, with no code changes required), the rate rose to 5.733 ± 0.621 — a **46% increase** over the uniform-DFR baseline. Correlated peak timing between the two operators matters a great deal; uncorrelated timing shape does not.

### 12.5 Loiter duration — linear

Loiter duration scales linearly with rate across the tested 5-30 minute range (weighted R² = 0.996), consistent with the other three structural parameters. This was cross-validated against three independently-collected data points (the Section 9 reference scenario, and two points from Sections 12.1-12.2), each matching within confidence intervals.

### 12.6 A single combined formula, validated across all four parameters

All four structural parameters (deliveries/day, total DFR events/day, route type via its duration ratio, and loiter duration) combine multiplicatively for **Scenario 3a**:

**rate_3a(radius) ≈ C(radius) × deliveries_per_day × total_DFR_events_per_day × loiter_minutes × route_type_multiplier**

where `route_type_multiplier` = 1.0 for out-and-back, ≈1.7 for triangular (the closed-form ratio of expected trip durations, Section 3.3), and C(radius) is a single fitted constant per footprint radius:

| Radius | C_3a (per delivery × DFR-event × loiter-min) | C_3b |
|---|---|---|
| 50m | 1.81e-6 | 8.33e-8 |
| 150m | 5.91e-6 | 5.75e-7 |
| 300m | 1.27e-5 | 1.98e-6 |
| 500m | 2.14e-5 | 5.31e-6 |

**Scenario 3b follows the same multiplicative structure** (same table, right column) with one caveat not yet independently verified: the Scenario 3b loiter-linearity assumption is carried over from Scenario 3a's validated result, not separately re-validated with its own dedicated high-powered sweep, since Scenario 3b's event counts are much lower, making a from-scratch validation more expensive.

**Scenario 3c does not depend on loiter duration at all** — it's governed by the DFR *transit* phase, which occurs entirely before loiter begins, so extending or shortening the subsequent loiter has no bearing on Scenario 3c's rate. Scenario 3c's own multiplicative constant has now been fitted (below): a preliminary two-axis fit showed poor agreement (15-23% relative difference between the n_deliveries-based and total-DFR-events-based estimates, driven by how rare Scenario 3c events are), so a dedicated higher-powered calibration was run and **all 28 available Scenario 3c data points across every phase of this sweep were pooled into a single joint weighted regression** rather than relying on two separate 1D fits. This gave:

**C_3c = 3.72e-6, weighted R² = 0.962** (per delivery × DFR-event, no radius or loiter dependence)

This constant is not radius-dependent (Scenario 3c uses the fixed 10m combined buffer, not the swept loiter-footprint radii) and sits at a plausible magnitude relative to Scenario 3a's own per-radius constants (between the 50m and 150m values) — a sensible order of magnitude given Scenario 3c's buffer size, though this is a plausibility observation, not independent validation. The route-type multiplier applies to Scenario 3c the same way as Scenario 3a/Scenario 3b (triangular/out-and-back ratio measured at 1.667x for Scenario 3c, consistent with the ~1.7x duration-ratio mechanism).

**Timing invariance is conditional**, per Section 12.4: this formula assumes DFR call timing stays uncorrelated with delivery timing (currently the model's default). If DFR timing is ever set to correlate with delivery peaks, a correction factor (empirically ≈1.46x for full peak-alignment in the tested case) would need to be layered on top.

### 12.6a Complete Master Formula, All Three Sub-Cases

With Scenario 3c's constant now fitted, all three sub-cases reduce to the same multiplicative family:

| Sub-case | Formula | Notes |
|---|---|---|
| Scenario 3a | `C_3a(radius) × del/day × DFR-events/day × loiter_min × route_mult` | radius-dependent, loiter-dependent |
| Scenario 3b | `C_3b(radius) × del/day × DFR-events/day × loiter_min × route_mult` | radius-dependent, loiter-dependent (loiter-linearity carried over from Scenario 3a, not independently re-validated) |
| Scenario 3c | `C_3c × del/day × DFR-events/day × route_mult` | **not** radius-dependent (fixed 10m buffer); **not** loiter-dependent (occurs pre-loiter) |

| Constant | Value |
|---|---|
| C_3a(50m) | 1.81e-6 |
| C_3a(150m) | 5.91e-6 |
| C_3a(300m) | 1.27e-5 |
| C_3a(500m) | 2.14e-5 |
| C_3b(50m) | 8.33e-8 |
| C_3b(150m) | 5.75e-7 |
| C_3b(300m) | 1.98e-6 |
| C_3b(500m) | 5.31e-6 |
| C_3c (radius-independent) | 3.72e-6 |
| route_mult | 1.0 (out-and-back) / ≈1.7 (triangular) |

### 12.7 Summary implication for a future interactive tool

**All four swept structural parameters behave multiplicatively**, so the entire swept parameter space (deliveries/day, dock count, calls/dock, loiter duration, route type) collapses to a small number of fitted constants (one C per radius per sub-case, one duration-ratio multiplier for route type) rather than requiring any dense grid at all. This considerably simplified the combined-tool build (Section 13).

---

## 13. Combined Model Architecture

Per explicit user direction, the two models (exposure-rate and fine-grained) are to become **one interactive tool**, not two requiring manual value copying. This section documents the integration design.

### 13.1 Data Flow

```
User inputs (shared across both layers)
        │
        ▼
Exposure-rate model (Layers 1+2)
  → K_3a (annualized rate)   → feeds fine-grained CRUISE-vs-orbit model (existing)
  → K_3b (annualized rate)   → feeds fine-grained STATIONARY-vs-orbit model (new, Section 13.2)
  → 3c (annualized rate)     → displayed only; advisory/contextual, NOT fed into any collision-rate calculation
        │
        ▼
Combined output: P(≥1 collision in one year) = 1 − (1−p_3a)^K_3a × (1−p_3b)^K_3b
  (computed and reported for both the sNMAC and contact thresholds)
```

**Shared parameters between layers:** the exposure model's footprint radius and the fine-grained model's orbit-radius extent (r_max) represent the same physical quantity and are now a **single shared slider** (50/150/300/500m), not two independently-settable values that could drift out of sync.

**Time horizon:** the combined tool's headline output is always the **annual** rate (K_3a, K_3b, and Scenario 3c are all annualized, consistent with the 365-operating-day assumption, Section 10). A daily rate is displayed alongside for interpretability, but is not itself the quantity used in the P(collision) calculation.

**K is shown as two numbers, plus a separate advisory number:** the interface displays K_3a and K_3b explicitly (not hidden), since they are the two quantities that actually drive the collision-rate calculation and are useful for sanity-checking. Scenario 3c is displayed as its own annualized count with a clear, explicit label that it is **contextual and advisory only** — not incorporated into the collision-rate output, per Section 7.1's original scope decision.

### 13.2 New Fine-Grained Sub-Model: Stationary Hover vs. Orbiting DFR (feeds K_3b)

The existing fine-grained model's entire geometry (Drone A transiting at cruise speed through Drone B's orbit) matches **Scenario 3a** directly and needed no change. It does **not** match **Scenario 3b** — during pickup/dropoff, Drone A is stationary (or descending) for the fixed 45-second hover, not transiting. A new sub-model was built: `hover_vs_orbit.py`.

## 14. Fine-Grained Model Correction: Removing Stacked Worst-Case Assumptions

The combined tool's reference scenario produced P(collision within one year) ≈ 99.7% at the sNMAC threshold — flagged as surprisingly high given the inputs. Investigation traced this to **stacked conservatism in the fine-grained per-encounter models** (`mc_prototype_orbit.py` for p_3a, `hover_vs_orbit.py` for p_3b), not to the exposure-rate side. Two specific worst-case assumptions were identified, corrected, and validated. Both original files are preserved as `*_v1_superseded.py`/`.json` for audit trail; the corrected versions are now the current files.

### 14.1 Fix #1 — Horizontal alignment (p_3a only)

**Before:** Drone A's flight path was assumed to always pass through the *exact center* of Drone B's orbit, every trial, with only a small Gaussian (TSE) offset on top — a genuine worst-case with no exceptions, inherited from Section 7's original alignment decision and never revisited when the model moved to orbit-based B motion.

**Why this was wrong:** the coarse exposure-rate layer only ever confirms "the route came within the footprint radius of the DFR location at some point" — never "passed through the exact center." Section 13.2 had already identified and fixed the identical problem for the hover case (p_3b), replacing "A at the orbit's exact center" with "A uniform within the footprint disk." The cruise case (p_3a) had the same flaw and was never corrected at the time.

**Fix:** A's cross-track offset from the orbit center is now drawn by sampling a point uniformly within the footprint disk (radius = r_max_m, matching the hover model's convention exactly) and using its cross-track component — this produces the mathematically correct semicircle-shaped marginal distribution (denser toward the center, tapering to zero at the edges), not a naive flat uniform. A small Gaussian is added on top, representing genuine navigation noise around wherever the true offset actually is.

**Validated:** the empirical standard deviation of the resulting offset (149.94) matched the theoretical prediction combining a semicircle distribution (std = R/2 = 150.0) with the small Gaussian in quadrature (150.02) almost exactly. Edge cases (zero/huge threshold) still behave correctly.

### 14.2 Fix #2 — Vertical placement (both p_3a and p_3b)

**Before:** both fine-grained models placed Drone B's active altitude sub-band worst-case-centered on Drone A's cruise altitude, narrowed to half the total declared vertical band — guaranteeing vertical overlap was always geometrically favorable to A.

**Why this was wrong:** the coarse exposure-rate model (`exposure_model.py`) never made this assumption — it draws DFR altitude uniformly across the *full* declared band, independent of delivery altitude, specifically so genuine vertical non-overlap remains possible (a requirement from early in that model's design, Section 5). The fine-grained models were never brought into agreement with this.

**Fix:** both fine-grained models now draw B's altitude directly from the full declared band (e.g., 100-300ft), with no narrowing or centering — matching the coarse model exactly.

**Validated:** edge cases (zero/huge threshold) still behave correctly in both models; the resulting probability reductions (below) are directionally and roughly quantitatively consistent with the analogous fix's known effect in the coarse model (which roughly halved the vertical pass rate under a similar correction).

### 14.3 Combined Effect — Before/After

At the standing reference point (drone size class 1, v_B=40mph, radius=300m):

| Quantity | Old (v1, stacked worst-case) | New (v2, corrected) | Ratio |
|---|---|---|---|
| p_3a (sNMAC) | 2.87e-3 | 1.30e-3 | 0.453x |
| p_3b (sNMAC) | 3.12e-3 | 1.47e-3 | 0.471x |

Both fixes together reduce the per-encounter probability by roughly half — consistent in magnitude with each fix independently being a real, non-trivial (not marginal) correction, not a token adjustment.

### 14.4 Reference Scenario, Recomputed

At the full reference scenario (1,000 deliveries/day, 3 docks × 8 calls/day, 15-min loiter, out-and-back, radius=300m, size class 1, v_B=45mph):

| | sNMAC | Contact |
|---|---|---|
| P(collision), 1 year — OLD | 99.75% | (not separately reported before) |
| P(collision), 1 year — NEW | **91.2%** | **31.9%** |
| Expected annual event count — NEW | 2.43 | 0.38 |

**The reduction is real but smaller than either fix's individual magnitude would suggest, and the reason is important:** K_3a (≈1,673/year) and K_3b (≈260/year) are computed entirely from the exposure-rate model (Sections 1-12), which neither fix touches — fixes #1 and #2 only ever affect the fine-grained per-encounter probabilities p_3a/p_3b, never K. The expected annual event count (K × p, summed across Scenario 3a and Scenario 3b) fell from ≈6 to ≈2.43 — a real ~2.5x reduction — but once an expected count is above roughly 2, "P(at least one)" stays high almost regardless of the exact value (1−e⁻² ≈ 86%, 1−e⁻²·⁴³ ≈ 91%), since the exponential saturates quickly. **The contact-threshold figure (31.9%) is more informative for judging the practical effect of these fixes**, since its lower baseline expected count (0.38) sits below the saturation region where small changes in the input still produce visible changes in the output.

**Open question, explicitly not resolved by this correction:** whether K itself (~1,900+ combined annual Scenario-3 encounters at these inputs) is a realistic figure for how often strategic conflict detection actually fails to prevent overlap at this operational scale. This is a question about the exposure-rate model's inputs and real-world deconfliction performance, not about fine-grained collision geometry, and remains open.

### 14.5 What Was Deliberately Not Changed

Per explicit instruction, the 365-operating-day annualization assumption (Section 10) was **not** revisited as part of this correction, despite being flagged earlier as a candidate lever — it is being treated as a standard, defensible convention for this domain, not an adjustable parameter to tune the headline number toward a preferred outcome.

---

## 15. Per-Operation Scenario 1/2/3 Probability (Tool Update)

Added to the combined tool alongside the existing annualized K readouts. Where K_3a, K_3b, and the Scenario 3c rate answer "how many such events are expected per year," this addition answers a complementary question: **for one single, randomly-chosen delivery flight, what's the chance it experiences a Scenario 3 overlap at all — versus being fully handled by Scenario 1 or Scenario 2?**

### 15.1 Calculation

Since K_3a_daily, K_3b_daily, and the Scenario 3c daily rate are already computed from the multiplicative formula (Section 12.6a), no new modeling work was needed — only a division and a combination step:

- `rule3a_per_op = K_3a_daily / deliveries_per_day`
- `rule3b_per_op = K_3b_daily / deliveries_per_day`
- `rule3c_per_op = 3c_daily / deliveries_per_day`
- `rule3_per_op = 1 − (1 − rule3a_per_op)(1 − rule3b_per_op)(1 − rule3c_per_op)`
- `rule1_or_2_per_op = 1 − rule3_per_op`

### 15.2 Scope decision: Scenario 3c is included here, unlike in the collision-probability calculation

This required one explicit scoping choice, flagged rather than assumed: **Scenario 3, for this calculation, includes Scenario 3c.** Section 1 originally defined Scenario 3 as having three sub-cases (Scenario 3a, Scenario 3b, Scenario 3c) — Scenario 3c was only ever excluded from the *collision-probability* formula (Section 13, "Combined Output") because no fine-grained collision geometry model exists for it, not because it fails to qualify as a Scenario 3 overlap. Since this new figure asks "did the safety net miss at all," not "what's the collision risk," Scenario 3c's inclusion is the more faithful reading of the original definition. This means the per-operation Scenario 3 figure and the collision-probability calculation now deliberately use two different scopes of "Scenario 3" — documented here explicitly so it doesn't read as an inconsistency.

### 15.3 A structural finding worth noting: per-operation probability is independent of delivery volume

Because K_3a_daily (and K_3b_daily, 3c_daily) scale linearly with deliveries/day (Section 12.1), dividing by deliveries/day cancels that dependence exactly:

`rule3a_per_op = C_3a(radius) × total_DFR_events_per_day × loiter_minutes × route_multiplier`

— deliveries/day drops out entirely. This is a real, sensible property, not a coincidence of the specific reference scenario: **a single delivery flight's odds of encountering a Scenario 3 overlap depend only on how active the DFR side is and on the encounter geometry — not on how many other deliveries are happening that same day.** Validated across the full slider range (including all sliders at their maximum simultaneously), the combined figure stays comfortably below 100% with no edge-case breakdown.

### 15.4 Reference scenario result

At the standing reference scenario (1,000 deliveries/day, 3 docks × 8 calls/day, 15-min loiter, out-and-back, 300m radius):

| Quantity | Value |
|---|---|
| Scenario 3a per operation | 0.458% |
| Scenario 3b per operation | 0.071% |
| Scenario 3c per operation | 0.009% |
| **Scenario 3 overlap, combined, per operation** | **0.538%** |
| **Scenario 1 or Scenario 2 (no overlap), per operation** | **99.462%** |

In plain terms: at these settings, roughly 1 in 186 delivery flights experiences some form of Scenario 3 overlap; the remaining ~99.46% are fully handled by strategic deconfliction.

---

## 16. Fine-Grained Model Reconciliation (Cross-Reference)

Two cross-model consistency gaps were found and resolved in the companion fine-grained collision model spec — logged here as a pointer, since this model's own conventions (the five-value discrete cruise-altitude distribution, Section 3.4; the no-vertical-gate rationale for Scenario 3b, Section 5) are what the fine-grained model was brought into alignment with:

- **Cruise-altitude scope match:** the fine-grained model previously computed collision probability at a single fixed cruise altitude, not this model's five-value distribution. Now resolved via conditioning the fine-grained Scenario 3a calculation on this model's own coarse vertical gate. See companion spec Section 3.
- **Hover vertical-sweep match:** the fine-grained model previously checked a single fixed hover altitude, not modeling the descent/ascent sweep that is this model's own justification for exempting Scenario 3b from any vertical gate (Section 5). Now resolved by modeling Drone A's hover-phase altitude as time-varying in the fine-grained model. See companion spec Section 4b.

Both p_3a and p_3b (the per-encounter probabilities feeding the combined tool's K_3a/K_3b compounding) increased as a result of these fixes — a counterintuitive but validated direction, explained in the companion spec. A related future-work item (modeling the approach-to-hover chord and its correlation with the subsequent hover as one joint encounter, rather than two independent draws) was identified but not built — see companion spec Section 10.
