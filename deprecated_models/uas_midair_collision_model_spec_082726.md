# UAS Midair Collision Probability Model — Working Spec

**Status:** Living document. Assumptions below reflect discussion-to-date and are expected to change as we iterate. This update (a) fixes a duplicate section-numbering error, (b) backports two corrections that were applied to the underlying model code but had only been documented in the companion exposure-rate spec, (c) adds the stationary-hover sub-model that this document had never described despite it being part of the same fine-grained collision-probability family, and (d) surfaces two new cross-model inconsistencies found while reconciling this document against the Layer 1+2 exposure-rate spec — flagged as open items, not silently resolved.

**Scope:** This model estimates the probability of a midair collision between two UAS whose ASTM F3548-compliant operational intent volumes overlap in space and time, despite both operators being otherwise strategically deconflicted. It does **not** address ground risk. It assumes the strategic deconfliction process itself failed or degraded to produce an unintended overlap — this model quantifies the residual risk *given* that overlap, not the probability of the overlap occurring in the first place.

---

## 1. Scenario Definition

Two UAS are considered:

- **Drone A ("Package Delivery"):** Transits through its volume at a known/bounded cruise speed. Its volume is a time-bounded extruded polygon (per ASTM F3548) representing Total System Error (TSE) in 3D around a planned trajectory. The volume's temporal extent (`n` minutes) is wider than A's actual flight time because A's exact takeoff time was not precisely known when the volume was checked for deconfliction.
- **Drone B ("Public Safety"):** Dispatched to a location and loiters within a cylindrical volume of known radius, maneuvering to observe a scene from multiple angles. B's specific path within the volume is not known in advance.

An "overlap" event means both aircraft are operating normally and within their own declared volumes, but two volumes that *should* have been strategically deconflicted are not — i.e., we are modeling residual/normal-operations risk, not risk from either operator flying out of conformance.

Two encounter geometries are modeled, corresponding to the two ways Drone A's flight phase can intersect Drone B's operation (Scenario 3a and Scenario 3b in the companion exposure-rate spec):
- **Cruise-vs-orbit** (Section 3–9a below): Drone A is transiting at cruise speed. Feeds Scenario 3a.
- **Stationary-hover-vs-orbit** (Section 4b below): Drone A is stopped for a pickup/dropoff. Feeds Scenario 3b.

---

## 2. Collision / Near-Miss Geometry

### 2.1 Criterion type
Cylindrical criterion (horizontal radius + independent vertical half-height), **not** spherical Euclidean distance. This was a deliberate change from an initial spherical assumption, specifically to correctly capture "one drone striking the side of the other" regardless of relative orientation — a concern raised directly by regulator feedback.

### 2.2 Threshold construction (additive: physical contact + safety buffer)

For a given pair of drones, each with a declared max horizontal dimension and max vertical dimension (height):

- **Horizontal contact radius** = (A_h_max / 2) + (B_h_max / 2)
- **Horizontal sNMAC radius (S_h)** = Horizontal contact radius + 5 m buffer
- **Vertical contact half-height** = (A_v_max / 2) + (B_v_max / 2)
- **Vertical sNMAC half-height (S_v)** = Vertical contact half-height + 1 m buffer

**Buffer provenance:**
- Horizontal 5 m buffer: anchored to prior sNMAC research (~15 ft).
- Vertical 1 m buffer: proposed for consistency with the horizontal approach; no independent literature anchor yet identified. Flagged as an assumption, not a citation-backed constant.

**Size range used for the Option A geometry lookup table:**

| | Max horizontal dim | Max vertical dim (height) |
|---|---|---|
| Smallest drone | 10 in (0.254 m) | 3 in (0.076 m) |
| Largest drone (e.g., package delivery) | 3 m | 0.5 m |

Worked reference points:
- Smallest + smallest: contact ≈ 0.25 m / 0.076 m → sNMAC ≈ 5.25 m / 1.08 m
- Largest + largest: contact = 3 m / 0.5 m → sNMAC = 8 m / 1.5 m
- Smallest + largest (small B vs. large A, the primary use case): contact ≈ 1.63 m / 0.29 m → sNMAC ≈ 6.63 m / 1.29 m

This geometry layer is pure arithmetic (no Monte Carlo dependency), so it is unaffected by either correction described in Section 7 / Section 4b below.

### 2.3 Presentation strategy
Rather than asserting one "true" threshold, the model computes P(collision) as a function of the threshold pair (S_h, S_v), reporting the full sensitivity curve/surface with two named reference points per drone-size-pair: physical contact and sNMAC. This makes the sNMAC number a **provable upper bound** on physical collision probability by construction (P is monotonic in threshold size), which is a defensible framing for a regulator rather than a single unexplained number.

A background lookup table sweeping combinations of A/B size pairs → (S_h, S_v) is maintained (`snmac_thresholds()` in `mc_prototype.py`), feeding both the cruise and hover sub-models' reference grids.

---

## 3. Position/Motion Model — Drone A

- **Along-track position:** Deterministic, constant cruise speed (v_A) once airborne, along the trajectory centerline.
  - **Simplification, explicitly flagged:** real-world speed variability exists and is not modeled. No adjustment applied; noted as a known gap.
- **Takeoff time:** Uniformly distributed across the volume's `n`-minute temporal margin. **Vestigial** — see Section 6; retained here only as historical context for why the along-track/timing setup looks the way it does.
- **Cross-track position — CORRECTED.** The original version fixed A's flight path to pass through the *exact center* of Drone B's orbit every trial (a hard worst-case with no exceptions; see the old Section 7 language, now superseded below). This is now understood to have been too extreme: the coarse exposure-rate layer only ever confirms "the route came within the footprint radius of the DFR location," never "passed exactly through center." The corrected model instead draws a point uniformly within the **footprint disk** (radius = `r_max_m`, the same shared-radius value used by the coarse exposure-rate model) and uses its cross-track component as A's offset from the orbit center. This produces the mathematically correct **semicircle-shaped marginal distribution** (denser toward the center, tapering to zero at the edges) — not the old fixed-at-zero assumption, and not a naive flat uniform either. A small Gaussian is added on top of this offset, representing genuine navigation/positioning noise around wherever the path's true large-scale offset actually is.
  - **Consequence for what W_A "means":** W_A (and the resulting σ = W_A/1.96) now represents only the *small-scale* TSE noise layered on top of the large-scale footprint-disk offset — not the full extent of A's positional uncertainty the way it did before this correction. The large-scale uncertainty is now governed by `r_max_m` (the shared radius parameter), not by W_A.
  - **Validated:** the empirical standard deviation of the resulting cross-track offset matched the theoretical prediction (semicircle distribution std = R/2, combined in quadrature with the small Gaussian) to within rounding at 500,000-trial precision.
- **Vertical position — CORRECTED (Item 2 / Approach D).** The Gaussian noise term itself (dz_A, sigma = H_A/1.96) around whatever nominal cruise altitude is in play is unchanged by any correction — but which nominal altitude that is has changed. The original version used a single fixed cruise altitude (200ft in all reference runs), while the coarse exposure-rate model varies Drone A's cruise altitude across **five discrete values** (200/250/300/350/400 ft) and applies its own vertical gate (|DFR_alt − cruise_alt| ≤ 50ft) when deciding whether a coarse Scenario 3a event occurs at all. Computing p_3a at a single, most-favorable-of-five altitude did not represent "the average Scenario 3a event" the coarse model actually produces.
  - **Fix:** cruise altitude is now drawn from the same five-value set, and the identical coarse gate is applied as a **rejection filter** — trials that would not have passed the coarse gate at all are excluded from both the numerator and denominator, not counted as non-collisions. This reproduces the correct Bayesian-conditional distribution of cruise altitude given a coarse Scenario 3a event, without needing to hand-derive posterior weights (which are not uniform: 200/250ft dominate at 40% each, 300ft at 20%, 350/400ft at 0% — the ±50ft coarse margin makes higher altitudes progressively, then completely, unable to ever pass).
  - **Validated:** the empirical coarse-gate pass rate (25.2% across 200,000 trials) matched the hand-computed prediction (25.0%, the weighted-average pass probability across the five values) almost exactly.
  - **Counterintuitive but correct result:** this correction *increased* p_3a relative to the old single-fixed-altitude approach (e.g., at the reference size/speed/radius point, roughly 0.0012 → 0.0027 at the sNMAC threshold) — the opposite of what a "less conservative" framing might predict. The mechanism: conditioning on the coarse gate concentrates B's altitude closer to A's cruise altitude than an unconditional full-band draw would, which makes the much tighter fine-grained vertical threshold (a few feet, vs. the coarse gate's 50ft) *more* likely to also be satisfied, not less. This is a real consequence of correctly modeling the conditional relationship, not an error — and a useful reminder that "adding realism" doesn't always move a number in the direction intuition suggests.
  - **Containment semantics (for the Gaussian noise term itself, unaffected by the above):** The declared half-width (W_A) and half-height (H_A) represent a **95% containment bound**, not a hard truncation.
  - σ derived via: **σ = declared half-dimension / 1.96** (95% two-sided Gaussian bound), applied **per-axis marginally** (horizontal 95% and vertical 95% are independent statements, not a joint 95% event).
  - A small residual tail (~5%, per axis) extends beyond the declared operational volume. This is intentional and should be described to reviewers as a deliberate, named modeling choice — not an oversight.
- **Aspect ratio guidance for sweep design (not a truncation rule):** realistic (W_A, H_A) pairs are expected to concentrate around (3–7 m) × (3–7 m); combinations like 1×1 or 10×10 are less likely but plausible; extreme ratios like 1×10 or 10×1 are unlikely. This informs where to concentrate resolution in the eventual sweep/heatmap grid, not the shape of the distribution itself. Given the note above, this guidance now bears specifically on the *small-scale* noise component, not on A's total positional spread.

---

## 4. Position/Motion Model — Drone B — HISTORICAL, SUPERSEDED

**This entire section describes the original dwell-and-reposition model, fully replaced by the continuous-orbit model in Section 4a. Retained verbatim for audit-trail context, not as current guidance.**

- **Horizontal position:** Strictly uniform over the disk of radius R_B (representing "a high degree of randomness" in B's positioning — no centerline or center-weighting, unlike A).
- **Vertical position:** Uniform over a sub-band spanning half of the total declared volume height, worst-case-centered on Drone A's nominal altitude.
- **Motion model:** B holds a fixed position for a dwell period, then redraws independently (uniform over disk × vertical sub-band) — "Path 1" in the historical Path 1/Path 2 discussion.

---

## 4a. Revised Drone B Motion Model: Continuous Orbit

The dwell-and-reposition model above has been replaced with a **continuous circular orbit** model, reflecting a more accurate description of how a public-safety drone actually behaves while observing a scene from multiple angles:

- Drone B flies a circle of radius **r** around a fixed center point at constant tangential speed **v_B** (fixed per scenario; range of interest 30–60 mph).
- **r varies from orbit to orbit**, swept over a real-world range with a **fixed 50m floor** (`r_min_m = 50`) and a sweepable ceiling (`r_max_m`, shared with the coarse exposure-rate model's footprint radius: 50/150/300/500m). Within a single trial, r is drawn once via a **size-biased sampling distribution**: pdf(r) ∝ r over [r_min, r_max]. This is a deliberate, documented correction: since orbit period = 2πr/v_B is proportional to r at fixed speed, a randomly-timed transit by Drone A is proportionally more likely to occur during a longer (larger-radius) orbit than a shorter one — an instance of the general "inspection paradox" (the same effect behind "why does the bus always feel late"). Sampling r uniformly rather than size-biased would systematically under-weight large orbits and bias the result.
- **Altitude — CORRECTED.** The original version placed B's active altitude sub-band worst-case-centered on Drone A's cruise altitude, narrowed to half the total declared band — guaranteeing vertical overlap was always geometrically favorable to A. The coarse exposure-rate model never made this assumption (it draws DFR altitude uniformly across the *full* declared band, independent of delivery altitude, specifically so genuine vertical non-overlap remains possible). The fine-grained model is now corrected to match: **B's altitude is drawn uniformly across the full declared band, with no narrowing or centering.** Altitude is fixed for the duration of one orbit and may change orbit to orbit.
- Drone B's **phase** (angular position at the instant Drone A crosses the orbit center's along-track coordinate) is drawn uniformly over [0, 2π) per trial.

**Two real crossings can occur within a single trial.** A straight line through the interior of a circle intersects it at exactly two points. If Drone B's angular speed (ω = v_B/r) is large enough relative to how long Drone A spends near the orbit center (governed by slow v_A, large r), B can sweep past both crossing points *within a single trial's brief encounter window* — not just across the ensemble of many trials. This was explicitly identified as a required modeling capability (not merely a documentation nuance) and is handled by evaluating B's true circular motion continuously through the encounter window (see Section 9a), rather than approximating B's motion as a straight tangent line (which was considered and rejected, since it cannot represent more than one crossing per trial).

**Structural consequence:** Drone A's takeoff-time uncertainty (n) and Drone B's total loiter duration / dwell period (T_B, dwell_seconds) no longer affect single-encounter P(collision) under this model and have been removed as inputs. See Section 6 for the full reasoning.

This orbit model also **subsumes the deferred Path 1/Path 2 comparison**: the orbit model *is* a continuous-motion model, specific to orbital behavior rather than generic random-walk motion. The general Path 1 vs. Path 2 bias question is considered resolved for the orbiting-B case specifically, though a generic (non-orbital) continuous-motion comparison remains unbuilt if ever needed for a different B behavior pattern.

---

## 4b. Alternate Sub-Model: Stationary Hover vs. Orbiting B (feeds Scenario 3b)

**Not previously documented in this file** — added here for completeness, since it is part of the same fine-grained collision-probability family (implemented in `hover_vs_orbit.py`) and the combined tool depends on it equally alongside the cruise model.

**Why a separate sub-model is needed:** the cruise model's entire geometry (Drone A transiting at cruise speed through Drone B's orbit) matches Scenario 3a directly. It does **not** match Scenario 3b — during pickup/dropoff, Drone A is stationary (or descending) for a fixed 45-second hover, not transiting. This requires different geometry: a fixed point vs. an orbiting circle, not a moving line vs. an orbiting circle.

**Drone A's hover position:** uniform within the **footprint-radius disk** (same `r_max_m`-shared convention as Section 3's corrected cross-track offset), plus a small isotropic Gaussian jitter (σ = W_A/1.96, GPS/hover-hold precision) on top. **Note on modeling history:** unlike the cruise model, this uniform-disk placement was *not* a later correction for the hover sub-model — an initial attempt did place A at the orbit's exact center (mirroring the cruise model's original, since-corrected assumption), but this was caught and fixed *before* the hover model was ever shipped, because placing A at the center combined with the 50m orbit-radius floor made collision structurally impossible (B is always ≥50m from center; A's small Gaussian jitter alone could never close that gap). This fix is what the later cruise-model correction (Section 3) was modeled on, not the reverse.

**Exposure window:** the fixed 45-second hover duration, not derived from any encounter geometry (unlike the cruise case, where the window is derived from orbit radius and cruise speed via `(r + S_h) / v_A`).

**Altitude — CORRECTED (Item 3).** The original version checked a single fixed hover altitude against a normal ±S_v vertical gate — a materially different (and more restrictive) check than the coarse exposure-rate model's own justification for exempting Scenario 3b from any vertical gate at all: the drone (or its payload) physically sweeps through the full altitude range while descending to deliver/collect the package and climbing back to cruise altitude, so it is very likely to pass through wherever the DFR happens to be at some point during the 45-second window.

**Fix:** Drone A's altitude is now modeled as **time-varying** — a linear descent from cruise altitude (drawn uniformly from the same five discrete values as the coarse model, with no rejection filter, since the coarse model applies no vertical gate at all for Scenario 3b) down to ground level (0 ft AGL) over the first half of the 45-second hover, then a linear ascent back to cruise altitude over the second half. Ground level as the sweep minimum is a deliberate, conservative choice (the widest plausible sweep) rather than an unvalidated assumption of a smaller minimum descent altitude — flagged explicitly as a simplification (see Section 11).

**Why this needed more than "add a time-varying check":** once altitude varies with time, a collision requires horizontal AND vertical proximity **at the same instant** — checking each independently and ANDing the results (valid under the old fixed-altitude version, since the vertical condition was then time-invariant) is no longer correct. The fix solves for the vertical-eligible time window(s) **exactly, in closed form** — possible because the altitude profile is piecewise-linear, not transcendental. Up to two such windows exist per trial (one during descent, one during ascent, since a V-shaped altitude profile can cross a fixed target band at most once per leg). The existing two-stage horizontal search is then restricted to those exact windows rather than the full 45-second duration — which also improves horizontal resolution for free, since the windows are typically far shorter than the full duration.

**Solving method, updated:** B's circular motion remains transcendental (as in the cruise model), so the two-stage coarse-then-refine search from Section 9a is still used for the horizontal dimension — now applied within the closed-form vertical windows rather than across the whole duration.

**Validated, and this is the more important validation given what changed:** checked against an independent brute-force method evaluating the **joint** horizontal-AND-vertical condition directly on a 20,000-point dense grid across the full window, with no closed-form shortcuts at all. **100% agreement, zero disagreements across 3,000 random trials, at both the sNMAC and contact thresholds.** This is a stronger check than validating horizontal and vertical separately (as the original two-stage method was validated), since the joint-timing correctness is exactly what changed here.

**Result:** p_3b also *increased* relative to the old fixed-altitude approach (e.g., at the reference point, roughly 0.0017 → 0.0018 at the sNMAC threshold — a smaller increase than Item 2's, but the same direction), consistent with modeling genuine vertical exposure across a swept range rather than a single-altitude snapshot.

**Documentation cleanup, resolved:** the module docstring previously described the original, since-corrected "A at orbit center" design even after the code was fixed to use uniform-disk placement. This has been corrected as part of this update — the docstring in the current file now accurately describes the implemented model.

---

## 5. Drone B Stationarity Assumption (Q4 origin) — HISTORICAL

Originally proposed as "B stationary relative to A's cruise speed," this was superseded by the Path 1 dwell/reposition model (Section 4, itself now historical), which was in turn fully superseded by the continuous orbit model (Section 4a). Retained for audit-trail context only.

---

## 6. Temporal Model: Why `n` and `T_B` Are Not Used

Two temporal quantities were considered earlier in this model's development and are **not used** in the current version:

- **n** — Drone A's takeoff-time uncertainty window.
- **T_B** — Drone B's total loiter/mission duration.

**Why they were dropped:** under the current orbiting-B motion model (Section 4a), Drone B's angular phase at the moment Drone A transits is drawn uniformly at random, independent of when A actually flies. That single uniform draw already fully captures "we don't know the timing relationship between the two aircraft." Separately modeling *when* A takes off (n) on top of this would be redundant — it would not change the distribution of outcomes, since the phase draw already marginalizes over all possible timing relationships. Similarly, T_B (how long B loiters in total) has no mechanism left to act through: the earlier dwell-period model made T_B meaningful by controlling how many independent B-position draws occurred during a loiter, but the orbit model has no dwell periods — B's state during a single encounter is fully described by its current orbit (radius, phase, altitude), not by how long it has been or will be loitering overall. Both parameters were therefore removed rather than kept as inert placeholders.

**Duration in this model is instead represented by K** (the independent-encounter count), computed in the companion exposure-rate model and fed into this model's output — the correct tool for the actual question of interest (cumulative fleet-scale risk across many encounters), as established when T_B was first found to be a poor duration axis.

**A's transit time remains a derived, not fixed, quantity.** It is a function of A's cruise speed (v_A) and the geometry of the encounter — specifically the along-track extent of Drone B's orbit, which spans up to r on either side of the orbit center **regardless of A's own cross-track offset** (a straight line always intersects a circle's along-track shadow the same way, independent of how far off-center the line sits — it's B's own along-track range, not A's offset, that bounds the search window). This gives:

**A's transit time through the shared candidate window ≈ (2 × r) / v_A**

(where r is the orbit radius active during that trial). This is used directly in the orbit model's candidate-window calculation (Section 9a) and varies trial-to-trial as r varies — it is not a fixed value. This formula's justification has been updated (it no longer depends on the now-superseded "worst-case center-crossing" framing — see Section 7), but the formula itself is unchanged, since it was already governed by B's geometry, not A's offset.

---

## 7. Horizontal Alignment Model — CORRECTED

**Original assumption (now superseded):** A's straight-line transit path was fixed to always cross through the exact center of B's orbit (i.e., along a full diameter, with zero cross-track offset before adding TSE noise). This was adopted as a deliberate worst-case encounter geometry, on the reasoning that any lesser chord (off-center offset) produces a smaller intersection and is therefore "less bad."

**Why this was corrected:** this was a genuine worst-case with *no exceptions*, which is more extreme than what the coarse exposure-rate layer actually confirms (it only knows the route came within the footprint radius at some point, never that it passed exactly through center). The hover sub-model (Section 4b) had already been built with a more calibrated uniform-disk placement from the start; the cruise model's still-more-extreme assumption was caught during investigation of an unexpectedly high combined-tool result and brought into alignment.

**Current model:** A's cross-track offset is drawn from a point sampled uniformly within the footprint disk (radius = `r_max_m`), using only its cross-track component (giving the correct semicircle-shaped marginal — see Section 3), plus a small Gaussian representing TSE noise. This:

- Removes "% volumetric overlap" as an independent free input — it is no longer a separate axis to sweep.
- Replaces the old fixed-alignment framing with a genuine distribution over how close A's path actually comes to B's orbit center, bounded by the shared footprint radius.
- Reduces single-encounter P(collision) relative to the old assumption — measured at roughly half, at the reference drone-size/speed/radius combination (see the companion exposure-rate spec's Section 14 for the full before/after validation).

**Radius range — CORRECTED.** Earlier text here referenced "R_B = 100–500m" as the corrected realistic sweep range (itself a correction from an original illustrative 30m). This has since been superseded again: the current model and the combined tool use a **fixed 50m floor** (`r_min_m = 50`) with a **sweepable ceiling** (`r_max_m` ∈ {50, 150, 300, 500}), shared directly with the coarse exposure-rate model's footprint radius parameter. The 100–500m range description was stale and has been corrected here; if this document is referenced elsewhere, check for the same stale figure.

---

## 8. Intended Output Structure

A 3D iso-surface (or heatmap family), with axes:

1. **Combined threshold, S** (paired S_h/S_v, varying together, derived from a swept A/B drone-size-pair lookup table per Section 2)
2. **Relative sizing of A's volume vs. B's cylinder** (W_A, H_A vs. r_max) — this axis's meaning is now understood through Section 7's corrected alignment model rather than the old fixed-center framing, but the basic idea (relative sizing drives overlap severity) still holds
3. **Duration — superseded.** T_B was originally proposed as this axis, but prototyping showed it has negligible effect on single-encounter P(collision) under the current model. See Section 8a for its replacement.

### 8a. Revised Axis 3: Independent Encounter Count (K)

**Motivating problem:** the real quantity of interest is not "one drone pair's single-encounter risk" but the **cumulative, fleet-scale risk of many delivery-drone/public-safety-drone encounters.** Single-encounter P(collision) (Sections 1–7's output) is the right *building block* for this, but is not itself the headline answer.

**Compounding model:** if each encounter is statistically independent, with single-encounter collision probability *p*, then across *K* independent encounters:

**P(at least one collision across K encounters) = 1 − (1 − p)^K**

K is now **computed by the companion exposure-rate model** (not swept as a free-standing parameter the way it originally was), via a validated multiplicative formula. See the companion spec's Sections 12–13 for the full derivation.

**Critical independence caveat — still applies:** K is only a faithful stand-in for "many different drone-pairs, many different encounters" when those encounters are genuinely statistically independent. It does **not** faithfully represent a single drone's outbound-plus-return trip past the same public-safety incident (those two transits are positively correlated, not independent) — this remains an open item (Section 10).

### 8b. Acceptability Threshold (TLOS) — Still Undefined

The model does not yet define what level of cumulative P(≥1 collision) constitutes "acceptable" risk. This is a deliberate, acknowledged gap — establishing a target level of safety (TLOS) is a separate regulatory/policy question, not a purely mathematical one.

### 8c. v_B Sweep Results and a Precision Caveat

Drone B's orbital speed was swept over its stated real-world range (30–60 mph), holding all else at reference values.

**sNMAC threshold:** single-encounter p increases clearly and monotonically with v_B, consistent with faster B sweeping more angular distance during A's fixed-duration transit window. This trend is well-resolved.

**Contact threshold:** the same trend is present in the point estimates but is **not well-resolved at 500,000 trials per point** — confidence intervals overlap substantially across the swept range. This is a real precision limitation, not a modeling error, and remains logged as an open item.

---

## 9. Computation Method — HISTORICAL (dwell-based description; see 9a for current)

**Monte Carlo simulation**, not closed-form (e.g., Reich model), selected deliberately. The description immediately below (algebraic per-dwell-period solving) applied to the original dwell-based B motion model (Section 4, now historical). See Section 9a for the solving method used by both current sub-models (cruise, Section 4a; hover, Section 4b), which is numerical rather than closed-form.

**Why Monte Carlo instead of a closed-form Reich-style derivation:** Reich's classical closed form relies on simple, standard paired distributions integrated against a fixed relative velocity in a stationary route structure. This problem mixes bounded-uniform and Gaussian distributions with orbital motion — a combination that is either intractable in closed form or requires special-function approximations harder to audit than direct simulation. Monte Carlo preserves every input distribution exactly as specified and keeps the pass/fail logic per trial fully transparent and auditable.

## 9a. Per-Trial Solving Method for the Orbit Model — Numerical, Not Closed-Form

Because B's true path is a circle, the horizontal distance between a fixed or moving Drone A and orbiting Drone B is a **transcendental function of time** — it has no general closed-form root, for *either* the cruise (Section 4a) or hover (Section 4b) sub-model. Both are solved numerically per trial using the same method:

1. **Coarse scan** (500 points) across the full candidate encounter window, locating the approximate minimum-distance point.
2. **Fine local refinement** (200 points) in a narrow window around that coarse minimum, to pin down the true minimum distance accurately without the cost of a single very-fine grid across the entire (much longer) encounter window.
3. Collision is registered if this refined minimum horizontal distance is ≤ S_h **and** the (time-invariant per trial) vertical separation is ≤ S_v.

**Validated, not merely assumed adequate:** checked against brute-force 5,000–8,000 point grids across thousands of random trials for both sub-models, including specifically constructed extreme cases chosen to maximize the chance of the fast method missing a genuine close approach (large r, slow-A/fast-B for the cruise model; large angular sweep for the hover model). Result: **100% agreement** on pass/fail collision classification across all validation trials, for both sub-models, at both threshold definitions. This validation should be re-run if the underlying speed or radius ranges are changed substantially.

---

## 10. Open Items / Not Yet Resolved

- **Delivery cruise-altitude scope mismatch — RESOLVED.** Was: the companion exposure-rate model varies Drone A's cruise altitude across five discrete values while this model used a single fixed altitude. Fixed via Approach D (Section 3): cruise altitude now drawn from the same five values, with the coarse model's own vertical gate applied as a rejection filter. Validated (coarse-gate pass rate matched hand-computed prediction). See Section 3 for full detail.
- **Hover vertical-sweep conceptual gap — RESOLVED.** Was: the coarse model exempts Scenario 3b from any vertical gate on the theory that the drone sweeps through the full altitude range during descent, while the fine-grained hover model checked only a single fixed altitude. Fixed by modeling Drone A's altitude as time-varying (linear descent-then-ascent) during hover, with the vertical-eligible windows solved exactly in closed form and the horizontal search restricted to them. Validated against an independent brute-force joint-condition check (100% agreement, 3,000 trials, both thresholds). See Section 4b for full detail.
- **Future work — joint Scenario 3a₁+3b encounter model (not built, model unchanged).** The current model treats every hover event (Scenario 3b) as if Drone A appears at the hover point directly, with no exposure counted for the transit into that point — that transit is currently absorbed into the ordinary Scenario 3a bucket, which assumes a full worst-case diameter crossing. This likely overstates combined 3a+3b risk in two compounding ways: (1) an approach segment ending inside the loiter disk is generally a shorter chord than a full-diameter transit, so today's full-diameter treatment overstates exposure for that specific sub-population; and (2) treating the approach (3a) and the subsequent hover (3b) as statistically independent, when they are actually the same continuous encounter, tends to overstate P(at least one collision) relative to a proper joint treatment. A future joint sub-model — modeling a continuous trial where Drone A transits partway in, then hovers, checked against one continuous B-orbit trajectory — would address both effects, but requires new coarse-model bookkeeping (to identify approach-to-hover vs. pure pass-through segments) and a new fine-grained joint sub-model, comparable in scope to originally building the hover sub-model. Logged as future work; the model is unchanged for now.
- **Monte Carlo precision at the contact threshold:** insufficient trials to resolve the v_B effect at the contact threshold (Section 8c) — CIs overlap across the swept range.
- **Correlated out-and-back transits (same job, same B loiter):** deferred. The independent-K compounding formula (Section 8a) is explicitly not valid here.
- **TLOS / acceptability threshold:** not defined (Section 8b).
- **A's transit-time formula refinement:** diameter/speed is the right functional form but should be reconciled against the actual threshold-crossing distance (a chord of length related to S_h, not just 2×r) during implementation.
- **Precise sweep ranges/resolution** for W_A, H_A, v_A: not yet finalized beyond the current reference values.
- **Linear descent/ascent profile (Section 4b):** a deliberate simplification for Item 3's fix; real descent profiles could be non-linear. Ground level (0ft) as the sweep minimum is likewise a conservative placeholder, not a validated figure for actual delivery operations.
- **Importance sampling investigation:** see Section 12c — logged as an open item, not abandoned.

---

## 11. Explicitly Logged Simplifications (Summary)

For quick reference, the deliberate simplifications currently in effect:

1. Drone A's speed is treated as constant/deterministic; real-world speed variability is not modeled.
2. A's declared operational volume is a 95%-containment bound on a Gaussian, not a hard boundary; ~5% per-axis tail probability exists outside the declared volume by design.
3. A's cross-track offset and hover position (Sections 3, 4b) are drawn from a footprint-disk-uniform distribution plus small Gaussian noise — a genuine distribution now, not a single worst-case point, but still bounded by the shared footprint radius rather than the full operational region.
4. B's altitude (both sub-models) is drawn uniformly across its full declared band, with no centering on A's altitude — corrected from the original worst-case-centered assumption (Sections 4a, 4b).
5. Drone A's cruise altitude is now drawn from the same five-value discrete distribution the coarse exposure-rate model uses, conditioned (for Scenario 3a) on the coarse model's own vertical gate — this is the Item 2 fix (Section 3), no longer a scope mismatch.
6. Drone A's hover-phase altitude is modeled as a linear descent-then-ascent sweep from cruise altitude to ground level and back — this is the Item 3 fix (Section 4b). The linear profile and the ground-level sweep minimum are both deliberate simplifications, not validated against real delivery-drone descent behavior.
7. The approach segment leading into a hover point is still treated identically to an ordinary pass-through Scenario 3a transit (full worst-case diameter crossing), rather than as a shorter, correlated chord terminating at the hover point — logged as future work (Section 10), not yet built.

---

## 12. Interactive 3D Tool — HISTORICAL, SUPERSEDED BY THE COMBINED TOOL

**This section describes the original standalone tool (`collision_probability_surface.html`). It has since been superseded by a combined tool spanning both this fine-grained model and the companion exposure-rate model — see the companion spec's Sections 13 and 15 for the current tool's architecture and the live per-operation probability display. Retained here for audit-trail context.**

`collision_probability_surface.html` rendered P(≥1 collision) over Drone B's physical size and orbital speed, with K as a manually-set slider (this document's original Section 8a design, before K was computed by the exposure-rate model). The K-axis was computed analytically in-browser from a precomputed grid of single-encounter probabilities — no Monte Carlo ran client-side.

### 12a. Tracked Placeholder Inputs — Contact Threshold Upper Bound

The contact-threshold surface used the **95% CI upper bound** from the Monte Carlo grid, not the point estimate — a deliberate conservative stand-in given the unresolved precision problem at this threshold. This convention **carries forward unchanged into the current combined tool.**

### 12b. Scope of the Original Standalone Tool

Held fixed: Drone A speed (45mph), Drone A physical size (3m/0.5m), Drone A's TSE half-width/half-height (5m/5m), Drone B's altitude band (100-300ft). The orbit radius extent (r_max) was a live slider (150-500m at the time; the shared radius control now also includes 50m, per Section 7's correction). A light/dark theme toggle was included and **carries forward unchanged into the current combined tool.**

### 12c. Importance Sampling Investigation — Logged as an Open Item, Not Abandoned

An attempt to map the reweighting/effective-sample-size methodology from Arun & Sachs, "Complexity-Aware System Validation for Autonomous Unmanned Aerial Systems" (their Eq. 9 self-normalized estimator and Eq. 10 Kish effective sample size) onto this problem's continuous-variable setting is implemented in `mc_prototype_orbit_IS.py`. The mapping itself was validated correct (the self-normalized estimator reproduces the plain-MC answer within CI at moderate bias levels). However, the specific biasing scheme tried (von Mises-distributed orbital phase toward a geometric closest-approach target, plus a truncated-normal vertical bias) did **not** improve precision at equal computational cost for the contact threshold — weight variance from the approximate phase-target formula outweighed the gain from more raw hits. This remains an open item, not a dead end: candidates for improvement include refining the phase-target formula to better match the true numerically-solved closest approach, or switching to a stratified-sampling approach instead of a smooth biased proposal. **Note:** this investigation predates, and was not repeated against, the corrected model in Sections 3/4a/4b/7 — if revisited, it should be re-run against the current (corrected) baseline, not the original.
